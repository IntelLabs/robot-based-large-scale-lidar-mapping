/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: PrescalerMsg.msg
 +Description: Message definition for prescaler settings, including vertical and horizontal prescaler values and state.
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
*/
#Prescaler
#Vertical  Default 1024 Options 512, 256, 0
#Horizontal    Default 40960

float32 pre_vertical
float32 pre_horizontal
int32 state