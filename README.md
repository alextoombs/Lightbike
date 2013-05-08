Lightbike
=========

The MIT License (MIT)

Copyright (c) <2013> <Patrick Bowlds, Benjamin Coffey, Michael Mellitt, Jacob Thordahl, Alexander Toombs>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Senior Design Lightbike project, ND EE 2013

Team website:  http://seniordesign.ee.nd.edu/2013/Design%20Teams/lightbike/index.html
(Board schematics also available at team website, subject to MIT License)

Embedded software for a battery charger for our senior design project.  

Designed to charge a stack of six Optima YellowTop D51 Batteries for use on an electric motorcycle.

Designed to be programmed onto a PIC32MX695F512H microcontroller, but anything in those families (5xx, 6xx, 7xx) should work.

Programmed using MPLAB IDE 8.1 and a Pickit 3.

Features:
-Feedback from ACS713 Current Sensor and several resistor dividers
-Output of power MOSFETs dynamically controlled by an MCP4706 8-bit DAC
-Now with more comments, and less spot welding of batteries!  (MOSFETs start closed)
-Light indicators on board display charging status even when I2C debug interface not installed
-NC relays 
