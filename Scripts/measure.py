# SPDX-FileCopyrightText: 2020 Dan Halbert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# Connect to an "eval()" service over BLE UART.

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

ble = BLERadio()

uart_connection = None

while True:
    if not uart_connection:
        print("Trying to connect...")
        for adv in ble.start_scan(ProvideServicesAdvertisement):
            if UARTService in adv.services:
                uart_connection = ble.connect(adv)
                print("Connected")
                break
        ble.stop_scan()

    if uart_connection and uart_connection.connected:
        uart_service = uart_connection[UARTService]
        uart_service.write(bytes([1,17]))
        while uart_connection.connected:
            for i in range(100):
                byte_array = uart_service.read(11)
                if byte_array is not None:
                    int_array = [x for x in byte_array]
                    if (int_array[10] != 0):
                        byte_array = uart_service.read(int_array[10])
                        int_array = int_array+[x for x in byte_array]
                    print(int_array)
