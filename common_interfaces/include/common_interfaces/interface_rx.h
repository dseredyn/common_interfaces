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

#ifndef COMMON_INTERFACES_INTERFACE_RX_H__
#define COMMON_INTERFACES_INTERFACE_RX_H__

#include "shm_comm/shm_channel.h"

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/Component.hpp"
#include "rtt/Logger.hpp"
#include <rtt/extras/SlaveActivity.hpp>

using namespace RTT;

template <class Container>
class InterfaceRx: public RTT::TaskContext {
public:
    explicit InterfaceRx(const std::string& name) :
        TaskContext(name, PreOperational),
        shm_name_("TODO"),
        buf_prev_(NULL),
        event_port_(false),
        port_msg_out_("msg_OUTPORT", false)
    {
        this->ports()->addPort(port_msg_out_);

        this->addOperation("pushBackPeerExecution", &InterfaceRx::pushBackPeerExecution, this, RTT::ClientThread)
            .doc("enable HW operation");

        this->addOperation("getDiag", &InterfaceRx::getDiag, this, RTT::ClientThread);

        addProperty("channel_name", param_channel_name_);
        addProperty("event_port", event_port_);
    }

    std::string getDiag() {
    // this method may not be RT-safe
        if (diag_buf_valid_) {
            //std::stringstream ss;
            //ros::message_operations::Printer<Container >::stream(ss, "", diag_buf_);
            //return ss.str();
            return "<data ok>";
        }
        return "<no data>";
    }

    bool pushBackPeerExecution(const std::string &peer_name) {
        Logger::In in("InterfaceRx::pushBackPeerExecution");
        if (isConfigured() || isRunning()) {
            Logger::log() << Logger::Warning << "this operation should be invoked before configure"
                          << Logger::endl;
            return false;
        }
        return true;
    }

    bool configureHook() {
        Logger::In in("InterfaceRx::configureHook");

        if (param_channel_name_.empty()) {
            Logger::log() << Logger::Error << "parameter channel_name is empty" << Logger::endl;
            return false;
        }

        Logger::log() << Logger::Info << "parameter channel_name is set to: \'" << param_channel_name_ << "\'" << Logger::endl;
        Logger::log() << Logger::Info << "parameter event_port is set to: " << (event_port_?"true":"false") << Logger::endl;

        shm_name_ = param_channel_name_;

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
            Logger::log() << Logger::Warning << "shm_connect_reader: could not create reader" << Logger::endl;
            create_channel = true;
        }

        if (!create_channel) {
            void *pbuf = NULL;
            result = shm_reader_buffer_get(re_, &pbuf);
            if (result < 0) {
                Logger::log() << Logger::Warning << "shm_reader_buffer_get: error: " << result << Logger::endl;
                create_channel = true;
            }
        }

        if (create_channel) {
            result = shm_create_channel(shm_name_.c_str(), sizeof(Container), 1, true);
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

        mTriggerOnStart = false;

        return true;
    }

    void cleanupHook() {
        shm_release_reader(re_);
    }

    bool startHook() {
        void *pbuf = NULL;

        int result = shm_reader_buffer_get(re_, &pbuf);
        if (result < 0) {
            Logger::log() << Logger::Error << "shm_reader_buffer_get: error: " << result << Logger::endl;
            return false;
        }

        buf_prev_ = reinterpret_cast<Container*>( pbuf );

        receiving_data_ = false;

        diag_buf_valid_ = false;
        diag_buf_ = Container();

        if (event_port_) {
            trigger();
        }

        return true;
    }

    void updateHook() {
    //*
        void *pbuf = NULL;
        Container *buf = NULL;

        bool buffer_valid = false;

        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ++ts.tv_sec;
        buffer_valid = (shm_reader_buffer_timedwait(re_, &ts, &pbuf) == 0);

    /*/
        Container *buf = reinterpret_cast<Container*>( reader_buffer_get(&re_) );
    //*/

        if ( buffer_valid
            && ((buf = reinterpret_cast<Container*>( pbuf )) != buf_prev_) )
        {
                // save the pointer of buffer
                buf_prev_ = buf;

                diag_buf_ = *buf;
                diag_buf_valid_ = true;
                // write received data to RTT port
                port_msg_out_.write(*buf);

                receiving_data_ = true;
        }
        else {
            receiving_data_ = false;
            diag_buf_valid_ = false;
        }

        if (event_port_) {
            trigger();
        }
    }

private:

    // properties
    std::string param_channel_name_;
    bool event_port_;
    
    std::string shm_name_;

    shm_reader_t* re_;
    Container *buf_prev_;
    bool receiving_data_;

    RTT::OutputPort<Container > port_msg_out_;

    Container diag_buf_;
    bool diag_buf_valid_;
};


#endif  // COMMON_INTERFACES_INTERFACE_RX_H__

