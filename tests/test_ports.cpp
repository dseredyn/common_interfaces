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

#include "test_ports.h"

using common_interfaces_test_msgs::Container;
using common_interfaces_test_msgs::SubContainer;
using common_interfaces_test_msgs::SubSubContainer;

namespace common_interfaces_test_interface {

//
// SubSubContainer_Ports interface
//
template <template <typename Type> class T >
SubSubContainer_Ports<T>::SubSubContainer_Ports(RTT::TaskContext &tc, const std::string &prefix, SubSubContainer SubContainer::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<SubSubContainer > >(new Port<T, uint32_t, SubSubContainer, SubSubContainer::_field1_type>(tc, prefix + "_field1", &SubSubContainer::field1)), &SubSubContainer::field1_valid);
    addPort(boost::shared_ptr<PortInterface<SubSubContainer > >(new Port<T, uint32_t, SubSubContainer, SubSubContainer::_field2_type>(tc, prefix + "_field2", &SubSubContainer::field2)));
}

//
// SubContainer_Ports interface
//
template <template <typename Type> class T >
SubContainer_Ports<T >::SubContainer_Ports(RTT::TaskContext &tc, const std::string &prefix, SubContainer Container::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<SubContainer > >(new SubSubContainer_Ports<T >(tc, prefix + "_sub1", &SubContainer::sub1)), &SubContainer::sub1_valid);
    addPort(boost::shared_ptr<PortInterface<SubContainer > >(new SubSubContainer_Ports<T >(tc, prefix + "_sub2", &SubContainer::sub2)));
    addPort(boost::shared_ptr<PortInterface<SubContainer > >(new Port<T, double, SubContainer, SubContainer::_field1_type >(tc, prefix + "_field1", &SubContainer::field1)), &SubContainer::field1_valid);
    addPort(boost::shared_ptr<PortInterface<SubContainer > >(new Port<T, double, SubContainer, SubContainer::_field2_type >(tc, prefix + "_field2", &SubContainer::field2)));
}

//
// Container_Ports interface
//
template <template <typename Type> class T>
Container_Ports<T >::Container_Ports(RTT::TaskContext &tc)
{
    addPort(boost::shared_ptr<PortInterface<Container > >(new SubContainer_Ports<T >(tc, "cont1", &Container::cont1)), &Container::cont1_valid);
    addPort(boost::shared_ptr<PortInterface<Container > >(new SubContainer_Ports<T >(tc, "cont2", &Container::cont2)));
    addPort(boost::shared_ptr<PortInterface<Container > >(new Port<T, SubContainer, Container, Container::_cont3_type >(tc, "cont3", &Container::cont3)), &Container::cont3_valid);
    addPort(boost::shared_ptr<PortInterface<Container > >(new Port<T, SubContainer, Container, Container::_cont4_type >(tc, "cont4", &Container::cont4)));
    addPort(boost::shared_ptr<PortInterface<Container > >(new Port<T, uint32_t, Container, Container::_field1_type >(tc, "field1", &Container::field1)), &Container::field1_valid);
    addPort(boost::shared_ptr<PortInterface<Container > >(new Port<T, uint32_t, Container, Container::_field2_type >(tc, "field2", &Container::field2)));
}

};  // namespace common_interfaces_test_interface

