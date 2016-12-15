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

#ifndef COMMON_INTERFACES_MESSAGE_CONCATE_H__
#define COMMON_INTERFACES_MESSAGE_CONCATE_H__

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/Logger.hpp"

using namespace RTT;

template <class Interface >
class MessageConcate: public RTT::TaskContext {
public:
    typedef typename Interface::Container_ Container;

    explicit MessageConcate(const std::string& name) :
        TaskContext(name, PreOperational),
        in_(this),
        port_msg_out_("msg_OUTPORT")
    {
        this->ports()->addPort(port_msg_out_);
        this->addOperation("getDiag", &MessageConcate::getDiag, this, RTT::ClientThread);
    }

    std::string getDiag() {
        if (diag_buf_valid_) {
            //std::stringstream ss;
            //ros::message_operations::Printer<Container >::stream(ss, "", diag_buf_);
            //return ss.str();
            return "<data ok>";
        }
        return "<no obligatory data>";
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        bool read_successful = in_.read(msg_);
        if (read_successful) {
            port_msg_out_.write(msg_);
            diag_buf_ = msg_;
        }
        diag_buf_valid_ = read_successful;
    }

private:

    Interface in_;

    Container msg_;
    RTT::OutputPort<Container > port_msg_out_;

    Container diag_buf_;
    bool diag_buf_valid_;
};


#endif  // COMMON_INTERFACES_MESSAGE_CONCATE_H__

