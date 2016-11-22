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

#ifndef __INTERFACE_PORTS_H__
#define __INTERFACE_PORTS_H__

#include <cstring>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include "rtt/RTT.hpp"

#include "common_interfaces/interface_port_data.h"

namespace interface_ports {

template <typename innerT, typename rosT >
class PortData {
public:
    PortData() {}

    void convertFromROS(const rosT &data) {
        data_.convertFromROS(data);
    }

    void convertToROS(rosT &data) {
        data_.convertToROS(data);
    }

    innerT& getDataRef() {
        return data_.data_;
    }

protected:
    PortRawData<innerT, rosT > data_;
};

template <template <typename Type> class T, typename innerT >
class PortOperation { };

template <typename innerT >
class PortOperation<RTT::InputPort, innerT> {
public:
    PortOperation(RTT::TaskContext &tc, const std::string &port_name) :
        port_(port_name + "_INPORT")
    {
        tc.ports()->addPort(port_);
    }

    bool readPorts(innerT &data) {
        return port_.read(data) == RTT::NewData;
    }

    void setDataSample(innerT &data) {
        // no operation for input port
    }

    void setName(const std::string& name) {
        port_.setName(name);
    }

    const std::string& getName() const {
        return port_.getName();
    }

protected:
    RTT::InputPort<innerT > port_;
};

template <typename innerT >
class PortOperation<RTT::OutputPort, innerT> {
public:
    PortOperation(RTT::TaskContext &tc, const std::string &port_name) :
        port_(port_name + "_OUTPORT", false)
    {
        tc.ports()->addPort(port_);
    }

    bool writePorts(innerT &data) {
        port_.write(data);
        return true;
    }

    void setDataSample(innerT &data) {
        port_.setDataSample(data);
    }

    void setName(const std::string& name) {
        port_.setName(name);
    }

    const std::string& getName() const {
        return port_.getName();
    }

protected:
    RTT::OutputPort<innerT > port_;
};

template <typename rosC >
class PortInterface {
public:
    virtual void convertFromROS(const rosC &container) = 0;
    virtual void convertToROS(rosC &container) = 0;
    virtual bool readPorts() = 0;
    virtual bool writePorts() = 0;
};

template <template <typename Type> class T, typename innerT, typename rosC, typename rosT >
class Port : public PortInterface<rosC > { };

template <typename innerT, typename rosC, typename rosT >
class Port<RTT::InputPort, innerT, rosC, rosT > : public PortInterface<rosC > {
public:
    Port(RTT::TaskContext &tc, const std::string &port_name, rosT rosC::*ptr) :
        po_(tc, port_name),
        data_(),
        ptr_(ptr)
    {
        po_.setDataSample(data_.getDataRef());
    }

    virtual void convertFromROS(const rosC &container) {
        data_.convertFromROS(container.*ptr_);
    }

    virtual void convertToROS(rosC &container) {
        data_.convertToROS(container.*ptr_);
    }

    virtual bool readPorts() {
        return po_.readPorts(data_.getDataRef());
    }

    virtual bool writePorts() {
        return false;
    }

    virtual void setName(const std::string& name) {
        po_.setName(name);
    }

    virtual const std::string& getName() const {
        return po_.getName();
    }


protected:

    PortOperation<RTT::InputPort, innerT> po_;

    PortData<innerT, rosT > data_;
    rosT rosC::*ptr_;
};

template <typename innerT, typename rosC, typename rosT >
class Port<RTT::OutputPort, innerT, rosC, rosT > : public PortInterface<rosC > {
public:
    Port(RTT::TaskContext &tc, const std::string &port_name, rosT rosC::*ptr) :
        po_(tc, port_name),
        data_(),
        ptr_(ptr)
    {
        po_.setDataSample(data_.getDataRef());
    }

    virtual void convertFromROS(const rosC &container) {
        data_.convertFromROS(container.*ptr_);
    }

    virtual void convertToROS(rosC &container) {
        data_.convertToROS(container.*ptr_);
    }

    virtual bool readPorts() {
        return false;
    }

    virtual bool writePorts() {
        return po_.writePorts(data_.getDataRef());
    }

    virtual void setName(const std::string& name) {
        po_.setName(name);
    }

    virtual const std::string& getName() const {
        return po_.getName();
    }

protected:

    PortOperation<RTT::OutputPort, innerT> po_;

    PortData<innerT, rosT > data_;
    rosT rosC::*ptr_;
};

template <typename rosC, typename rosT >
class PortsContainer : public PortInterface<rosC > {
public:
    typedef boost::shared_ptr<PortInterface<rosT > > PortInterfacePtr;

    PortsContainer(rosT rosC::*ptr) :
        ptr_(ptr)
    {}

    virtual bool readPorts() {
        for (int i = 0; i < ports_.size(); ++i) {
            valid_vec_[i] = ports_[i].first->readPorts();
        }
        return true;
    }

    virtual bool writePorts() {
        for (int i = 0; i < ports_.size(); ++i) {
            if (valid_vec_[i]) {
                ports_[i].first->writePorts();
            }
        }
        return true;
    }

    virtual void convertFromROS(const rosC &ros) {
        for (int i = 0; i < ports_.size(); ++i) {
            ports_[i].first->convertFromROS(ros.*ptr_);
            if (ports_[i].second != NULL) {
                valid_vec_[i] = ros.*ports_[i].second;
            }
            else {
                valid_vec_[i] = true;
            }
        }
    }

    virtual void convertToROS(rosC &ros) {
        for (int i = 0; i < ports_.size(); ++i) {
            ports_[i].first->convertToROS(ros.*ptr_);
            if (ports_[i].second != NULL) {
                (ros.*ports_[i].second) = valid_vec_[i];
            }
        }
    }

    void addPort(PortInterfacePtr port, uint8_t rosC::*ptr = NULL) {
        ports_.push_back( std::make_pair(port, ptr) );
        valid_vec_.push_back(false);
    }

private:
    std::vector<std::pair<PortInterfacePtr, uint8_t rosC::*>  > ports_;
    std::vector<bool > valid_vec_;
    rosT rosC::*ptr_;
};

class ContainerOuter {
public:
};

template <typename rosC >
class PortsContainerOuter : public PortInterface<rosC >, public ContainerOuter {
public:
    typedef boost::shared_ptr<PortInterface<rosC > > PortInterfacePtr;

    PortsContainerOuter()
    {}

    virtual bool readPorts() {
        for (int i = 0; i < ports_.size(); ++i) {
            valid_vec_[i] = ports_[i].first->readPorts();
        }
        return true;
    }

    virtual bool writePorts() {
        for (int i = 0; i < ports_.size(); ++i) {
            if (valid_vec_[i]) {
                ports_[i].first->writePorts();
            }
        }
        return true;
    }

    virtual void convertFromROS(const rosC &ros) {
        for (int i = 0; i < ports_.size(); ++i) {
            ports_[i].first->convertFromROS(ros);
            if (ports_[i].second != NULL) {
                valid_vec_[i] = ros.*ports_[i].second;
            }
            else {
                valid_vec_[i] = true;
            }
        }
    }

    virtual void convertToROS(rosC &ros) {
        for (int i = 0; i < ports_.size(); ++i) {
            ports_[i].first->convertToROS(ros);
            if (ports_[i].second != NULL) {
                (ros.*ports_[i].second) = valid_vec_[i];
            }
        }
    }

    void addPort(PortInterfacePtr port, uint8_t rosC::*ptr = NULL) {
        ports_.push_back( std::make_pair(port, ptr) );
        valid_vec_.push_back(false);
    }

private:
    std::vector<std::pair<PortInterfacePtr, uint8_t rosC::*>  > ports_;
    std::vector<bool > valid_vec_;
};


};  // namespace interface_ports

#endif  // __INTERFACE_PORTS_H__

