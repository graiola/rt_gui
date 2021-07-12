/*
 * Copyright 2019 Gennaro Raiola
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Gennaro Raiola
 */

#ifndef RT_GUI_RT_GUI_H
#define RT_GUI_RT_GUI_H

namespace rt_gui
{

class RtGui
{
public:

    static RtGui& getGui()
    {
        static RtGui gui;
        return gui;
    }

    template <typename data_t>
    void add(const data_t& data, const std::string& data_name = "")
    {

    }

    void sync()
    {

    }

    void sync(const std::string& data_name)
    {

    }

private:

  RtGui()
  {

  }

  //~RtGui()

  RtGui(const RtGui&)= delete;
  RtGui& operator=(const RtGui&)= delete;

};


} // namespace


#endif
