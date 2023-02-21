# Copyright 2022 @RoadBalance
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import rclpy.node

class ParamExNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('param_ex_node')

        self.declare_parameter('string_param', 'world')
        self.declare_parameter('int_param', 119)
        self.declare_parameter('float_param', 3.1415)
        self.declare_parameter('arr_param', [1,2,3])
        self.declare_parameter('nested_param.string_param', 'Wee Woo')

        string_param = self.get_parameter('string_param')
        int_param = self.get_parameter('int_param')
        float_param = self.get_parameter('float_param')
        arr_param = self.get_parameter('arr_param')
        nested_param = self.get_parameter('nested_param.string_param')

        self.get_logger().info(f"\nstring_param: {string_param.value} \
            \nint_param: {int_param.value} \
            \nfloat_param: {float_param.value} \
            \narr_param: {arr_param.value} \
            \nnested_param.string_param: {nested_param.value}"
        )

def main():
    rclpy.init()

    node = ParamExNode()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()