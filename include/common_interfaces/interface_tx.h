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

#ifndef INTERFACE_TX_H__
#define INTERFACE_TX_H__

#include "shm_comm/shm_channel.h"

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/Component.hpp"
#include "rtt/Logger.hpp"

using namespace RTT;

template <template <template <typename Type> class RTTport> class Interface>
class InterfaceTx: public RTT::TaskContext {
public:
    typedef Interface<RTT::InputPort > InterfaceInport;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit InterfaceTx(const std::string& name) :
        TaskContext(name, PreOperational),
        in_(*this),
        shm_name_("TODO")
    {
        addProperty("channel_name", channel_name_);
    }

    bool configureHook() {
        Logger::In in("InterfaceTx::configureHook");

        if (channel_name_.empty()) {
            Logger::log() << Logger::Error << "channel_name is empty" << Logger::endl;
            return false;
        }

        shm_name_ = channel_name_;

        bool create_channel = false;

        int result = shm_connect_writer(shm_name_.c_str(), &wr_);
        if (result == SHM_INVAL) {
            Logger::log() << Logger::Error << "shm_connect_writer: invalid parameters" << Logger::endl;
            return false;
        }
        else if (result == SHM_FATAL) {
            Logger::log() << Logger::Error << "shm_connect_writer: memory error" << Logger::endl;
            return false;
        }
        else if (result == SHM_NO_CHANNEL) {
            Logger::log() << Logger::Warning << "shm_connect_writer: could not open shm object, trying to initialize the channel..." << Logger::endl;
            create_channel = true;
        }
        else if (result == SHM_CHANNEL_INCONSISTENT) {
            Logger::log() << Logger::Warning << "shm_connect_writer: shm channel is inconsistent, trying to initialize the channel..." << Logger::endl;
            create_channel = true;
        }
        else if (result == SHM_ERR_INIT) {
            Logger::log() << Logger::Error << "shm_connect_writer: could not initialize channel" << Logger::endl;
            return false;
        }
        else if (result == SHM_ERR_CREATE) {
            Logger::log() << Logger::Error << "shm_connect_writer: could not create reader" << Logger::endl;
            return false;
        }

        if (create_channel) {
            result = shm_create_channel(shm_name_.c_str(), sizeof(typename InterfaceInport::Container), 1, true);
            if (result != 0) {
                Logger::log() << Logger::Error << "create_shm_object: error: " << result << "   errno: " << errno << Logger::endl;
                return false;
            }

            result = shm_connect_writer(shm_name_.c_str(), &wr_);
            if (result != 0) {
                Logger::log() << Logger::Error << "shm_connect_writer: error: " << result << Logger::endl;
                return false;
            }
        }



        return true;
    }

    void cleanupHook() {
        shm_release_writer(wr_);
    }

    bool startHook() {
        Logger::In in("InterfaceTx::startHook");
        void *pbuf = NULL;
        if (shm_writer_buffer_get(wr_, &pbuf) < 0) {
            return false;
        }

        buf_ = reinterpret_cast<typename InterfaceInport::Container*>(pbuf);
        return true;
    }

    void stopHook() {
    }

    void updateHook() {
        uint32_t test_prev = cmd_out_.test;

        in_.readPorts();
        in_.convertToROS(cmd_out_);

        if (test_prev == cmd_out_.test) {
            Logger::In in("InterfaceTx::updateHook");
            Logger::log() << Logger::Warning << "executed updateHook twice for the same packet " << cmd_out_.test << Logger::endl;
            error();
        }
        else {
            //Logger::log() << Logger::Info << Logger::endl;
        }

        if (buf_ == NULL) {
            Logger::In in("InterfaceTx::updateHook");
            Logger::log() << Logger::Error << "writer get NULL buffer" << Logger::endl;
            error();
        }
        else {
            *buf_ = cmd_out_;
            shm_writer_buffer_write(wr_);
            //Logger::log() << Logger::Debug << "sending command" << Logger::endl;
        }
        void *pbuf = NULL;
        shm_writer_buffer_get(wr_, &pbuf);
        buf_ = reinterpret_cast<typename InterfaceInport::Container*>(pbuf);
    }


private:

    // properties
    std::string channel_name_;

    std::string shm_name_;
    shm_writer_t* wr_;

    InterfaceInport in_;
    typename InterfaceInport::Container cmd_out_;
    typename InterfaceInport::Container* buf_;
};

#endif  // INTERFACE_TX_H__

