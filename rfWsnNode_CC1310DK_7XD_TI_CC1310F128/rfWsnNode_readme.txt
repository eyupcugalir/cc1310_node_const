Example Summary
---------------
The WSN Node example illustrates how to create a Wireless Sensor
Network Node device which sends packets to a concentrator. This example is
meant to be used together with the WSN Concentrator example to form a one-
to-many network where the nodes send messages to the concentrator.

This examples showcases the use of several Tasks, Semaphores and Events to
get sensor updates and send packets with acknowledgement from the concentrator.
For the radio layer, this example uses the EasyLink API which provides an
easy-to-use API for the most frequently used radio operations.

For more information on the EasyLink API and usage refer to
http://processors.wiki.ti.com/index.php/SimpleLink-EasyLink

Peripherals Exercised
---------------------
Board_LED2            When the Node sends a packet, it also toggles Board_LED2.
Analog Light Sensor   Measured using the ADC by the SCE task

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
Run the example. On another board run the WSN Concentrator example.
This node should show up on the LCD of the Concentrator.

Application Design Details
--------------------------
This examples consists of two tasks, one application task and one radio
protocol task. It also consists of an Sensor Controller Engine, SCE, Task which
samples the ADC.

The ADC task on the SCE checks the ADC value once per second. If this value has
changed by a certain, maskable, amount since the last time it notified the CM3,
then it wakes it up again. If the change is less than the masked value, then
it does not wake up the CM3. A minimum wakeup period can also be configured so
that the CM3 is waken if no ADC changes greater than the masked value happen
within the timeout.

The NodeTask waits to be woken up by the SCE. When it wakes up it toggles
Board_LED1 and sends the new ADC value to the NodeRadioTask.

The NodeRadioTask handles the radio protocol. This sets up the EasyLink
API and uses it to send new ADC values to the concentrator. After each sent
packet it waits for an ACK packet back. If it does not get one, then it retries
three times. If it did not receive an ACK by then, then it gives up.

The default frequency is 868.0 MHz, in order to change the frequency, please see
"RadioProtocol.h". This can also be used to change the PHY settings to be either
the default IEEE 802.15.4g 50kbit, Long Range Mode or custom settings. In the
case of custom settings, the "smartrf_settings.c" file is used. This can be
changed either by exporting from Smart RF Studio or directly in the
file.