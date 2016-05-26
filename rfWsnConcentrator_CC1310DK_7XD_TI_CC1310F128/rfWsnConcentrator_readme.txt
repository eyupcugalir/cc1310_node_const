Example Summary
---------------
The WSN Concentrator example illustrates how to create a simple Wireless Sensor
Network Concentrator device which listens for packets from other nodes. This
example is meant to be used together with the WSN Node example to form a one-
to-many network where the nodes send messages to the concentrator.

This examples showcases the use of several Tasks, Semaphores and Events to
receive packets, send acknowledgements and display the received data on the
LCD. For the radio layer, this example uses the EasyLink API which provides
an easy-to-use API for the most frequently used radio operations.

For more information on the EasyLink API and usage refer to
http://processors.wiki.ti.com/index.php/SimpleLink-EasyLink

Peripherals Exercised
---------------------
LCD     When the collector receives data from a new node, it is given a new
        row on the display and the reveived value is shown. If more than 7
        nodes are detected, the device list rolls over, overriding the first.

        Whenever an updated value is received from a node, it is updated on
        the LCD display.

Resources & Jumper Settings
---------------------------
Please refer to the development board's specific "Settings and Resources"
section in the Getting Started Guide. For convenience, a short summary is also
shown below.

| Development board | Notes                                                  |
| ================= | ====================================================== |
| CC1310DK           |                                                          |
| ----------------- | ------------------------------------------------------ |

Example Usage
-------------
Run the example. On another board (or several boards) run the WSN Node example.
The LCD will show the discovered node(s).

Application Design Details
--------------------------
This examples consists of two tasks, one application task and one radio
protocol task.

The ConcentratorRadioTask handles the radio protocol. This sets up the EasyLink
API and uses it to always wait for packets on a set frequency. When it receives
a valid packet, it sends an ACK and then forwards it to the ConcentratorTask.

The ConentratorTask receives packets from the ConcentratorRadioTask, displays
the data on the LCD and toggles Board_LED1.

The default frequency is 868.0 MHz, in order to change the frequency, please see
"RadioProtocol.h". This can also be used to change the PHY settings to be either
the default IEEE 802.15.4g 50kbit, Long Range Mode or custom settings. In the
case of custom settings, the "smartrf_settings.c" file is used. This can be
changed either by exporting from Smart RF Studio or directly in the
file.