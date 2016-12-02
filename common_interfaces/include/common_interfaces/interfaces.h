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

#ifndef __COMMON_INTERFACES_INTERFACES_H__
#define __COMMON_INTERFACES_INTERFACES_H__

#include <rtt/Component.hpp>

#include "common_interfaces/interface_tx.h"
#include "common_interfaces/interface_rx.h"

#include "common_interfaces/message_split.h"
#include "common_interfaces/message_concate.h"

#define ORO_LIST_INTERFACE_COMPONENTS( CLASS_NAME_COMMAND, CLASS_NAME_STATUS, INTERFACE_PREFIX ) \
typedef InterfaceTx<CLASS_NAME_STATUS > INTERFACE_PREFIX##StatusTx; \
typedef InterfaceRx<CLASS_NAME_STATUS > INTERFACE_PREFIX##StatusRx; \
typedef InterfaceTx<CLASS_NAME_COMMAND > INTERFACE_PREFIX##CommandTx; \
typedef InterfaceRx<CLASS_NAME_COMMAND > INTERFACE_PREFIX##CommandRx; \
typedef MessageSplit<CLASS_NAME_COMMAND##_Ports > INTERFACE_PREFIX##CommandSplit; \
typedef MessageConcate<CLASS_NAME_STATUS##_Ports > INTERFACE_PREFIX##StatusConcate; \
typedef MessageSplit<CLASS_NAME_STATUS##_Ports > INTERFACE_PREFIX##StatusSplit; \
typedef MessageConcate<CLASS_NAME_COMMAND##_Ports > INTERFACE_PREFIX##CommandConcate; \
namespace INTERFACE_PREFIX##StatusTx_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##StatusTx) } \
namespace INTERFACE_PREFIX##StatusRx_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##StatusRx) } \
namespace INTERFACE_PREFIX##CommandTx_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##CommandTx) } \
namespace INTERFACE_PREFIX##CommandRx_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##CommandRx) } \
namespace INTERFACE_PREFIX##CommandSplit_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##CommandSplit) } \
namespace INTERFACE_PREFIX##StatusConcate_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##StatusConcate) } \
namespace INTERFACE_PREFIX##StatusSplit_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##StatusSplit) } \
namespace INTERFACE_PREFIX##CommandConcate_namespace { ORO_LIST_COMPONENT_TYPE(INTERFACE_PREFIX##CommandConcate) }

#endif  // __COMMON_INTERFACES_INTERFACES_H__

