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

#ifndef __COMMON_INTERFACES_INTERFACE_PORTS_H__
#define __COMMON_INTERFACES_INTERFACE_PORTS_H__

#include <cstring>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include "rtt/RTT.hpp"

#include "common_interfaces/interface_port_data.h"

namespace interface_ports {

template <typename rosC >
class PortInterface {
public:
    virtual void convertFromROS(const rosC &container) = 0;
    virtual void convertToROS(rosC &container) = 0;
    virtual bool readPorts() = 0;
    virtual bool writePorts() = 0;
};

template <template <typename Type> class T, typename innerT, typename rosT >
class Port : public PortInterface<rosT > { };

template <typename innerT, typename rosT >
class Port<RTT::InputPort, innerT, rosT > : public PortInterface<rosT > {
public:
    Port(RTT::TaskContext &tc, const std::string &port_name) :
        port_(port_name + "_INPORT"),
        data_()
    {
        tc.ports()->addPort(port_);
        //port_.setDataSample(data_.data_);     // no operation for input port
    }

    virtual void convertFromROS(const rosT &data) {
        data_.convertFromROS(data);
    }

    virtual void convertToROS(rosT &data) {
        data_.convertToROS(data);
    }

    virtual bool readPorts() {
        return port_.read(data_.data_) == RTT::NewData;
    }

    virtual bool writePorts() {
        return false;
    }

protected:

    RTT::InputPort<innerT > port_;

    PortRawData<innerT, rosT > data_;
};

template <typename innerT, typename rosT >
class Port<RTT::OutputPort, innerT, rosT > : public PortInterface<rosT > {
public:
    Port(RTT::TaskContext &tc, const std::string &port_name) :
        port_(port_name + "_OUTPORT", true),
        data_()
    {
        tc.ports()->addPort(port_);
        port_.setDataSample(data_.data_);
    }

    virtual void convertFromROS(const rosT &data) {
        data_.convertFromROS(data);
    }

    virtual void convertToROS(rosT &data) {
        data_.convertToROS(data);
    }

    virtual bool readPorts() {
        return false;
    }

    virtual bool writePorts() {
        port_.write(data_.data_);
    }

protected:

    RTT::OutputPort<innerT > port_;

    PortRawData<innerT, rosT > data_;
};

};  // namespace interface_ports

#endif  // __COMMON_INTERFACES_INTERFACE_PORTS_H__

