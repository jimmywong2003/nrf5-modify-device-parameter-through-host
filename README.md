# nrf5-modify-advertising-parameter

This example is to show how to connect with mobile phone to change the advertising content / parameter.

It adds the thingy configure service (tcs) with NUS and battery service in this example.

It can use the nRF Connect to change the following parameters.
1) Device name [R/W]
2) Advertising interval [R/W]
3) Connection parameter (minimum connection interval, maximum connection interval) [R/W]
4) FW version (Read only)
5) Configure the MTU length [R/W]

The detail description of this example will be posted on my personal blog later.

## Requirement

* NRF52832 DK
* IDE: Segger Embedded Studio
* SDK 15.3 / S132v6.1.1

## Note

This is the demo source code. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.

The application is built to be used with the official nRF5 SDK, that can be downloaded from developer.nordicsemi.com

You can place this example inside the SDK15.3/example/ble_peripheral folder.
