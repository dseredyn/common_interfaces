/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef INTERFACE_RX_H__
#define INTERFACE_RX_H__

#include "shm_comm/shm_channel.h"

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/Component.hpp"
#include "rtt/Logger.hpp"
#include <rtt/extras/SlaveActivity.hpp>

using namespace RTT;

template <template <template <typename Type> class RTTport> class Interface>
class InterfaceRx: public RTT::TaskContext {
public:
    typedef Interface<RTT::OutputPort > InterfaceOutport;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit InterfaceRx(const std::string& name) :
        TaskContext(name, PreOperational),
        shm_name_("TODO"),
        buf_prev_(NULL),
        out_(*this),
        event_port_(false)
    {
        this->ports()->addPort("command_OUTPORT", port_command_out_);

        this->addOperation("pushBackPeerExecution", &InterfaceRx::pushBackPeerExecution, this, RTT::ClientThread)
            .doc("enable HW operation");

        addProperty("channel_name", channel_name_);
        addProperty("event_port", event_port_);
    }

    bool pushBackPeerExecution(const std::string &peer_name) {
        Logger::In in("InterfaceRx::pushBackPeerExecution");
        if (isConfigured() || isRunning()) {
            Logger::log() << Logger::Warning << "this operation should be invoked before configure"
                          << Logger::endl;
            return false;
        }
        peer_list_.push_back(peer_name);
        return true;
    }

    bool configureHook() {
        Logger::In in("InterfaceRx::configureHook");

        if (channel_name_.empty()) {
            Logger::log() << Logger::Error << "channel_name is empty" << Logger::endl;
            return false;
        }

        shm_name_ = channel_name_;

        bool create_channel = false;

        int result = shm_connect_reader(shm_name_.c_str(), &re_);
        if (result == SHM_INVAL) {
            Logger::log() << Logger::Error << "shm_connect_reader: invalid parameters" << Logger::endl;
            return false;
        }
        else if (result == SHM_FATAL) {
            Logger::log() << Logger::Error << "shm_connect_reader: memory error" << Logger::endl;
            return false;
        }
        else if (result == SHM_NO_CHANNEL) {
            Logger::log() << Logger::Warning << "shm_connect_reader: could not open shm object, trying to initialize the channel..." << Logger::endl;
            create_channel = true;
        }
        else if (result == SHM_CHANNEL_INCONSISTENT) {
            Logger::log() << Logger::Warning << "shm_connect_reader: shm channel is inconsistent, trying to initialize the channel..." << Logger::endl;
            create_channel = true;
        }
        else if (result == SHM_ERR_INIT) {
            Logger::log() << Logger::Error << "shm_connect_reader: could not initialize channel" << Logger::endl;
            return false;
        }
        else if (result == SHM_ERR_CREATE) {
            Logger::log() << Logger::Error << "shm_connect_reader: could not create reader" << Logger::endl;
            return false;
        }

        if (create_channel) {
            result = shm_create_channel(shm_name_.c_str(), sizeof(typename InterfaceOutport::Container), 1, true);
            if (result != 0) {
                Logger::log() << Logger::Error << "create_shm_object: error: " << result << "   errno: " << errno << Logger::endl;
                return false;
            }

            result = shm_connect_reader(shm_name_.c_str(), &re_);
            if (result != 0) {
                Logger::log() << Logger::Error << "shm_connect_reader: error: " << result << Logger::endl;
                return false;
            }
        }



        TaskContext::PeerList l = this->getPeerList();
        if (peer_list_.size() != l.size()) {
            Logger::log() << Logger::Error << "peer_list_.size() != l.size()   "
                          << peer_list_.size() << " != " << l.size() << Logger::endl;
            return false;
        }

        for (std::list<std::string >::const_iterator it2 = peer_list_.begin(); it2 != peer_list_.end(); ++it2) {
            TaskContext *tc = NULL;
            bool found = false;
            for (TaskContext::PeerList::const_iterator it = l.begin(); it != l.end(); ++it) {
                if ( (*it) == (*it2)) {
                    tc = this->getPeer( (*it) );
                    break;
                }
            }
            if (tc == NULL) {
                Logger::log() << Logger::Error << "could not find peer "
                              << (*it2) << " in peer execution list" << Logger::endl;
                return false;
            }
            peers_.push_back(tc);
        }

        for (std::list<TaskContext* >::iterator it = peers_.begin(); it != peers_.end(); ++it) {
            (*it)->setActivity( new RTT::extras::SlaveActivity(this->getActivity(), (*it)->engine()));

            Attribute<bool> triggerOnStart = (*it)->attributes()->getAttribute("TriggerOnStart");
            if (!triggerOnStart.ready()) {
                Logger::log() << Logger::Error << "attribute TriggerOnStart of peer "
                              << (*it)->getName() << " is not ready" << Logger::endl;
                return false;
            }
            triggerOnStart.set(false);
        }

        mTriggerOnStart = false;

        return true;
    }

    void cleanupHook() {
        shm_release_reader(re_);

//        shm_remove_channel(shm_name_.c_str());
    }

    bool startHook() {
        void *pbuf = NULL;

        int result = shm_reader_buffer_get(re_, &pbuf);
        if (result < 0) {
            return false;
        }

        buf_prev_ = reinterpret_cast<typename InterfaceOutport::Container*>( pbuf );

        receiving_data_ = false;

        if (event_port_) {
            trigger();
        }

        return true;
    }

    void updateHook() {
    //*
        void *pbuf = NULL;
        typename InterfaceOutport::Container *buf = NULL;

        bool buffer_valid = false;
        if (receiving_data_) {
            timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;
            buffer_valid = (shm_reader_buffer_timedwait(re_, &ts, &pbuf) == 0);
        }
        else {
            buffer_valid = (shm_reader_buffer_get(re_, &pbuf) == 0);
        }

        if (buffer_valid) {
            buf = reinterpret_cast<typename InterfaceOutport::Container*>( pbuf );
        }
    /*/
        Container *buf = reinterpret_cast<Container*>( reader_buffer_get(&re_) );
    //*/

        if (!buffer_valid) {
    //        Logger::In in("InterfaceRx::updateHook");
    //        Logger::log() << Logger::Debug << "could not receive data (NULL buffer)" << Logger::endl;
            receiving_data_ = false;
        }
        else {
            if (buf != buf_prev_) {
                buf_prev_ = buf;
                port_command_out_.write(*buf);

                out_.convertFromROS(*buf);
                out_.writePorts();
                receiving_data_ = true;
            }
            else {
                receiving_data_ = false;
            }
        }

        for (std::list<TaskContext* >::iterator it = peers_.begin(); it != peers_.end(); ++it) {
            if (!(*it)->update()) {
                Logger::In in("InterfaceRx::updateHook");
                Logger::log() << Logger::Error << (*it)->getName() << "->update() has failed" << Logger::endl;
                error();
            }
        }

        if (event_port_) {
            trigger();
        }
    }

private:

    // properties
    std::string channel_name_;
    bool event_port_;
    
    std::string shm_name_;

    shm_reader_t* re_;
    typename InterfaceOutport::Container *buf_prev_;
    bool receiving_data_;

    InterfaceOutport out_;

    RTT::OutputPort<typename InterfaceOutport::Container > port_command_out_;

    std::list<std::string > peer_list_;
    std::list<TaskContext* > peers_;
};


#endif  // INTERFACE_RX_H__

