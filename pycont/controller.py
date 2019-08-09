"""
.. module:: controller
   :platform: Unix
   :synopsis: A module used for controlling the pumps.

.. moduleauthor:: Jonathan Grizou <Jonathan.Grizou@gla.ac.uk>

"""

# -*- coding: utf-8 -*-

import json
from typing import Tuple, List

from pycont.dtprotocol import DTInstructionPacket
from ._logger import create_logger
from . import pump_protocol

from SerialLabware.controllers import LabDevice

#: Represents the Broadcast of the C3000
C3000Broadcast = '_'

#: Switches the C3000 to a certain address
C3000SwitchToAddress = {
    '0': '1',
    '1': '2',
    '2': '3',
    '3': '4',
    '4': '5',
    '5': '6',
    '6': '7',
    '7': '8',
    '8': '9',
    '9': ':',
    'A': ';',
    'B': '<',
    'C': '=',
    'D': '>',
    'E': '?'
}

# Valve positions
VALVE_INPUT = 'I'
VALVE_OUTPUT = 'O'
VALVE_BYPASS = 'B'
VALVE_EXTRA = 'E'
VALVE_6WAY_LIST = ['1', '2', '3', '4', '5', '6']

#: Microstep Modes
MICRO_STEP_MODE_0 = 0
MICRO_STEP_MODE_2 = 2

# Number of steps per each Microstep Mode
N_STEP_MICRO_STEP_MODE_0 = 3000
N_STEP_MICRO_STEP_MODE_2 = 24000

# The maximum top velocities
MAX_TOP_VELOCITY_MICRO_STEP_MODE_0 = 6000
MAX_TOP_VELOCITY_MICRO_STEP_MODE_2 = 48000

# Default Input/Output (I/O) baud rate
DEFAULT_IO_BAUDRATE = 9600
# Default timeout for I/O operations
DEFAULT_IO_TIMEOUT = 0.1

# Specifies a time to wait
WAIT_SLEEP_TIME = 0.1
#: Sets the maximum number of attempts to Write and Read
MAX_REPEAT_WRITE_AND_READ = 10
#: Sets the maximum time to repeat a specific operation
MAX_REPEAT_OPERATION = 10

# DT Protocol
DTStart = '/'


class PumpCommunicationError(Exception):
    """
    Exception for when the communication with the pump fails.
    """
    def __init__(self):
        print('Communication error!')


class ControllerRepeatedError(Exception):
    """ Exception for when there has been too many repeat attempts. """
    pass


class PumpHWError(Exception):
    """
    Exception for when the pump encounters an hardware error.
    """
    def __init__(self, error_code='x', pump='unknown'):

        self.pump_name = pump
        self.error_code = error_code.lower()

        print(f"*** ERROR on pump {self.pump_name} ***")

        if self.error_code == 'a':
            print("Initialization failure!")
        elif self.error_code == 'b':
            print("Invalid command!")
        elif self.error_code == 'c':
            print("Invalid operand!")
        elif self.error_code == 'f':
            print("EEPROM failure!")
        elif self.error_code == 'g':
            print("Pump not initialized!")
        elif self.error_code == 'i':
            print("Plunger overload!")
        elif self.error_code == 'j':
            print("Valve overload!")
        elif self.error_code == 'k':
            print("Plunger stuck!")
        else:
            print("** ERROR ** Unknown error")


class C3000Controller(LabDevice):
    """
    This class represents the main controller for the C3000.
    The controller is what controls the pumps.
    This implementation assume a 1 to 1 correspondence between connections and pumps, to use multiple pumps on the same
    connection in a daisy chain use PyCont v1
    """
    def __init__(self, name, port, total_volume, micro_step_mode=MICRO_STEP_MODE_2, top_velocity=6000,
                 initialize_valve_position=VALVE_INPUT, baudrate=DEFAULT_IO_BAUDRATE, timeout=DEFAULT_IO_TIMEOUT,
                 address=None):
        self.logger = create_logger(self.__class__.__name__)

        # Handles both TCP-IP and Serial connections
        if address is not None or ':' in port:
            # TCP-IP
            if address:
                tcpip_port = port
            else:
                address, tcpip_port = port.split(':')
            parameters = {
                "baudrate": baudrate,
                "transmit_timeout": 0,
                "receive_timeout": timeout,
                "address": address,
                "port": tcpip_port
            }
            super().__init__(device_name="Tricont C3000 - "+name, connection_mode="tcpip", connection_parameters=parameters)
        else:
            # Serial
            parameters = {
                "baudrate": baudrate,
                "transmit_timeout": 0,
                "receive_timeout": timeout,
                "port": port
            }
            super().__init__(device_name="Tricont C3000 - "+name, connection_mode="serial", connection_parameters=parameters)

        self.command_terminator = "\r"
        self.reply_terminator = "\r"

        self.name = name
        self.address = self.autodiscover_address()
        self._protocol = pump_protocol.C3000Protocol(self.address)
        self.initialize_valve_position = initialize_valve_position

        self._microstep_mode = None
        self.micro_step_mode = micro_step_mode

        if self.micro_step_mode == MICRO_STEP_MODE_0:
            self.number_of_steps = int(N_STEP_MICRO_STEP_MODE_0)
        elif self.micro_step_mode == MICRO_STEP_MODE_2:
            self.number_of_steps = int(N_STEP_MICRO_STEP_MODE_2)
        else:
            raise ValueError(f"Microstep mode {self.micro_step_mode} is not handled")

        self.total_volume = float(total_volume)  # in ml (float)
        self.steps_per_ml = int(self.number_of_steps / self.total_volume)

        self._default_top_velocity = None
        self.default_top_velocity = top_velocity

    @classmethod
    def from_config(cls, pump_name, pump_config):
        """
        Obtains the configuration data.

        Args:
            cls (Class): The initialising class.

            pump_name (str): Name of the pump.

            pump_config (Dict): Dictionary containing the pump configuration data.

        Returns:
            C3000Controller: New C3000Controller object with the data set from the configuration.
        """
        pump_config['total_volume'] = float(pump_config['volume'])  # in ml (float)
        del(pump_config['volume'])

        return cls(pump_name, **pump_config)

    # Essentially a default_top_velocity setter
    def __setattr__(self, key, value):
        if key == "default_top_velocity":
            if self.validate_top_velocity(value) is False:
                value = 1000
        super(C3000Controller, self).__setattr__(key, value)

    def prepare_message(self, message: DTInstructionPacket):
        return message.to_string()

    def parse_reply(self, reply):
        return reply.rstrip().rstrip('\x03').lstrip(DTStart)

    def autodiscover_address(self):
        """ Find the address of the pump on the specified connection """
        for switch, address in C3000SwitchToAddress.items():
            reply = self.write_and_read_from_pump(pump_protocol.C3000Protocol(address).forge_report_status_packet())

            if reply in (pump_protocol.STATUS_IDLE_ERROR_FREE, pump_protocol.STATUS_BUSY_ERROR_FREE):
                self.logger.debug(f"Found a pump with dial on position {switch}!")
                return address
            elif reply in (pump_protocol.ERROR_STATUSES_IDLE or pump_protocol.ERROR_STATUSES_BUSY):
                self.logger.warning(f"Found a pump with dial on position {switch} with an error status!")
                return address

    def write_and_read_from_pump(self, packet) -> Tuple[str, str]:
        """
        Writes packets to and reads the response from the pump.

        Args:
            packet (DTInstructionPacket): The packet to be written.

        Returns:
            decoded_response (str): The decoded response.

        Raises:
            PumpIOTimeOutError: If the response time is greater than the timeout threshold.

        """

        # Send message
        self.send_message(packet, prepare=True)

        # Receive reply
        response = self.receive_reply(parse=True)

        if response:
            # Split response in components
            address = response[0]
            status = response[1]
            data = response[2:]

            self.logger.debug(f"Reply '{response}' decoded to Address {address} Status {status} Data {data}")
            return status, data
        else:
            self.logger.debug(f"Decode error for {response}! :(")
            raise PumpCommunicationError()

    def volume_to_step(self, volume_in_ml):
        """
        Determines the number of steps for a given volume.

        Args:
            volume_in_ml (float): Volume in millilitres.

        Returns:
            int(round(volume_in_ml * self.steps_per_ml))

        """
        return int(round(volume_in_ml * self.steps_per_ml))

    def step_to_volume(self, step):
        """
        Determines the volume in a specific step.

        Args:
            step (int): Step number.

        Returns:
            step / float(self.steps_per_ml

        """
        return step / float(self.steps_per_ml)

    def flowrate_to_speed(self, flowrate: float) -> float:
        """ Converts a flow rate in ml/min to a linear pump speed (in increments/seconds) """
        if self.micro_step_mode == MICRO_STEP_MODE_0:
            return self.total_volume * 6000 / flowrate
        elif self.micro_step_mode == MICRO_STEP_MODE_2:
            return self.total_volume * 48000 / flowrate
        else:
            raise ValueError("Invalid micro step settings")

    def speed_to_flowrate(self, speed: float) -> float:
        """ Converts linear pump speed (in increments/seconds) to a flow rate in ml/min """
        if self.micro_step_mode == MICRO_STEP_MODE_0:
            return self.total_volume / 6000 * speed
        elif self.micro_step_mode == MICRO_STEP_MODE_2:
            return self.total_volume / 48000 * speed
        else:
            raise ValueError("Invalid micro step settings")

    def is_ready(self):
        return self.is_idle()

    def is_idle(self):
        """
        Determines if the pump is idle or Busy

        Returns:
            True (bool): The pump is idle.

            False (bool): The pump is not idle.

        Raises:
            ValueError: Value returned from the pump is not valid.

        """
        report_status_packet = self._protocol.forge_report_status_packet()
        (status, _) = self.write_and_read_from_pump(report_status_packet)
        if status == pump_protocol.STATUS_IDLE_ERROR_FREE:
            return True
        elif status == pump_protocol.STATUS_BUSY_ERROR_FREE:
            return False
        elif status in pump_protocol.ERROR_STATUSES_BUSY:
            raise PumpHWError(error_code=status, pump=self.name)
        elif status in pump_protocol.ERROR_STATUSES_IDLE:
            raise PumpHWError(error_code=status, pump=self.name)
        else:
            raise ValueError('The pump replied status {}, Not handled'.format(status))

    def is_busy(self) -> bool:
        """ Determines if the pump is busy. """
        return not self.is_idle()

    def is_initialized(self) -> bool:
        """ Determines if the pump has been initialised. """
        initialized_packet = self._protocol.forge_report_initialized_packet()
        (_, init_status) = self.write_and_read_from_pump(initialized_packet)
        return bool(int(init_status))

    def initialise_device(self) -> None:
        """ Implementation of LabDevice abstract class. """
        self.smart_initialize()

    def smart_initialize(self, valve_position=None, secure=True) -> None:
        """
        Initialises the pump and sets all pump parameters.

        Args:
            valve_position (int): Position of the valve, default set None.
            secure (bool): Ensures that everything is correct, default set to True.
        """
        if not self.is_initialized():
            self.initialize(valve_position, secure=secure)
        self.init_all_pump_parameters(secure=secure)

    def initialize(self, valve_position=None, secure=True):
        """
        Initialises the pump.

        Args:
            valve_position (int): Position of the valve, default set to None.

            secure (bool): Ensures that everything is correct.

        Raises:
            ControllerRepeatedError: Too many failed attempts to initialise.

        """
        if valve_position is None:
            valve_position = self.initialize_valve_position

        self.initialize_valve_only()
        self.set_valve_position(valve_position, secure=secure)
        self.initialize_no_valve()

    def initialize_no_valve(self, operand_value=None, wait=True):
        """
        Initialise with no valves.

        Args:
            operand_value (int): Value of the supplied operand.

            wait (bool): Whether or not to wait until the pump is idle, default set to True.

        """

        if operand_value is None:
            if self.total_volume < 1:
                operand_value = 1  # Half plunger stall force for syringes with volume of 500 uL or less
            else:
                operand_value = 0

        self.write_and_read_from_pump(self._protocol.forge_initialize_no_valve_packet(operand_value))
        if wait:
            self.wait_till_ready()

    def initialize_valve_only(self, operand_string='0,0', wait=True):
        """
        Initialise with valves only.

        Args:
            operand_string (str): Value of the supplied operand.

            wait (bool): Whether or not to wait until the pump is idle, default set to True.

        """
        self.write_and_read_from_pump(self._protocol.forge_initialize_valve_only_packet(operand_string))
        if wait:
            self.wait_till_ready()

    def init_all_pump_parameters(self):
        """ Initialises the pump parameters, Microstep Mode, and Top Velocity. """
        self.microstep_mode = self._microstep_mode
        self.top_velocity = self.default_top_velocity

    @property
    def microstep_mode(self):
        """ Returns the current Microstep Mode. """
        return self._microstep_mode  # Faster than asking pump

    @microstep_mode.setter
    def microstep_mode(self, mode):
        """ Sets the Microstep Mode. """
        self._microstep_mode = mode
        self.write_and_read_from_pump(self._protocol.forge_microstep_mode_packet(mode))

    def validate_top_velocity(self, velocity: float) -> bool:
        """ Checks if the provided value is a valid velocity in the current step mode

        :param velocity: Target velocity
        """
        if self.micro_step_mode == MICRO_STEP_MODE_0:
            max_range = MAX_TOP_VELOCITY_MICRO_STEP_MODE_0
        elif self.micro_step_mode == MICRO_STEP_MODE_2:
            max_range = MAX_TOP_VELOCITY_MICRO_STEP_MODE_2
        else:
            raise ValueError(f"Current micro step mode not valid ({self.micro_step_mode})")

        return velocity in range(1, max_range + 1)

    def ensure_default_top_velocity(self):
        """ Ensures that the top velocity is the default top velocity. """
        if self.top_velocity != self.default_top_velocity:
            self.top_velocity = self.default_top_velocity

    @property
    def top_velocity(self):
        """ Gets the current top velocity. """
        top_velocity_packet = self._protocol.forge_report_peak_velocity_packet()
        (_, top_velocity) = self.write_and_read_from_pump(top_velocity_packet)
        return int(top_velocity)

    @top_velocity.setter
    def top_velocity(self, velocity):
        """ Sets the top velocity for the pump. """
        if self.validate_top_velocity(velocity):
            self.logger.debug(f"setting pump top velocity to {velocity}")
            self.write_and_read_from_pump(self._protocol.forge_top_velocity_packet(velocity))
        else:
            self.logger.warning(f"pump top velocity was NOT set to {velocity} as such value is not valid")

    @property
    def current_steps(self) -> int:
        """ Gets the current position of the plunger. """
        plunger_position_packet = self._protocol.forge_report_plunger_position_packet()
        (_, steps) = self.write_and_read_from_pump(plunger_position_packet)
        return int(steps)

    @property
    def remaining_steps(self) -> int:
        """ Gets the remaining number of steps. """
        return self.number_of_steps - self.current_steps

    @property
    def current_volume(self) -> float:
        """ Get the current syringe position in ml """
        return self.step_to_volume(self.current_steps)

    @property
    def remaining_volume(self):
        """ Gets the remaining volume that can be pumped (i.e. total-current). """
        return self.total_volume - self.current_volume

    def is_volume_pumpable(self, volume_in_ml) -> bool:
        """
        Determines if the volume is pumpable.

        Args:
            volume_in_ml (float): The supplied volume.
        """
        steps = self.volume_to_step(volume_in_ml)
        return steps <= self.remaining_steps

    def pump(self, volume_in_ml, from_valve=None, speed_in=None, wait=False, secure=True):
        """
        Sends the signal to initiate the pump sequence.

        .. warning:: Change of speed will last after the scope of this function but will be reset to default each time speed_in == None

        Args:
            volume_in_ml (float): Volume to pump (in mL).
            from_valve (chr): Pump using the valve, default set to None.
            speed_in (int): Speed to pump, default set to None.
            wait (bool): Waits for the pump to be idle, default set to False.
            secure (bool): Ensures everything is correct, default set to True.

        Returns:
            True (bool): The supplied volume is pumpable.
            False (bool): Supplied volume is not pumpable.

        """
        if self.is_volume_pumpable(volume_in_ml) is False:
            return False

        if speed_in is not None:
            self.top_velocity = speed_in
        else:
            self.ensure_default_top_velocity()

        if from_valve is not None:
            self.set_valve_position(valve_position=from_valve, secure=secure)

        steps_to_pump = self.volume_to_step(volume_in_ml)
        pump_packet = self._protocol.forge_pump_packet(steps_to_pump)
        self.write_and_read_from_pump(pump_packet)

        if wait:
            self.wait_till_ready()

        return True

    def is_volume_deliverable(self, volume_in_ml) -> bool:
        """
        Determines if the supplied volume is deliverable.

        Args:
            volume_in_ml (float): The supplied volume in mL.

        Returns:
            True (bool): The number of steps is <= the current steps.

            False (bool): The number of steps is > the current steps.

        """
        steps = self.volume_to_step(volume_in_ml)
        return steps <= self.current_steps

    def deliver(self, volume_in_ml, to_valve=None, speed_out=None, wait=False, secure=True):
        """
        Delivers the volume payload.

        .. warning:: Change of speed will last after the scope of this function but will be reset to default each time speed_out == None

        Args:
            volume_in_ml (float): The supplied volume to deliver.

            to_valve (chr): The valve to deliver the payload to, default set to None.

            speed_out (int): The speed of delivery, default set to None.

            wait (bool): Waits for the pump to be idle, default set to False.

            secure (bool): Ensures that everything is correct, default set to False.

        """
        if self.is_volume_deliverable(volume_in_ml) is False:
            return False

        if volume_in_ml == 0:
            return True

        if speed_out is not None:
            self.top_velocity = speed_out
        else:
            self.ensure_default_top_velocity()

        if to_valve is not None:
            self.set_valve_position(to_valve, secure=secure)

        steps_to_deliver = self.volume_to_step(volume_in_ml)
        deliver_packet = self._protocol.forge_deliver_packet(steps_to_deliver)
        self.write_and_read_from_pump(deliver_packet)

        if wait:
            self.wait_till_ready()

        return True

    def transfer(self, volume_in_ml, from_valve, to_valve, speed_in=None, speed_out=None):
        """
        Transfers the desired volume in mL.

        Args:
            volume_in_ml (float): The volume to transfer.

            from_valve (chr): The valve to transfer from.

            to_valve (chr): The valve to transfer to.

            speed_in (int): The speed of transfer to valve, default set to None.

            speed_out (int): The speed of transfer from the valve, default set to None.

        """
        volume_transferred = min(volume_in_ml, self.remaining_volume)
        self.pump(volume_transferred, from_valve, speed_in=speed_in, wait=True)
        self.deliver(volume_transferred, to_valve, speed_out=speed_out, wait=True)

        remaining_volume_to_transfer = volume_in_ml - volume_transferred
        if remaining_volume_to_transfer > 0:
            self.transfer(remaining_volume_to_transfer, from_valve, to_valve, speed_in, speed_out)

    def is_volume_valid(self, volume_in_ml) -> bool:
        """
        Determines if the supplied volume is valid.

        Args:
            volume_in_ml (float): The supplied volume.

        Returns:
            True (bool): The supplied volume is <= the total volume and >= 0

            False (bool): The supplied volume is > total volume or < 0

        """
        return 0 <= volume_in_ml <= self.total_volume

    def go_to_volume(self, volume_in_ml, speed=None, wait=False):
        """
        Moves the pump to the desired volume.

        .. warning:: Change of speed will last after the scope of this function but will be reset to default each time speed == None

        Args:
            volume_in_ml (float): The supplied volume.
            speed (int): The speed of movement, default set to None.
            wait (bool): Waits for the pump to be idle, default set to False.

        Returns:
            True (bool): The supplied volume is valid.
            False (bool): THe supplied volume is not valid.

        """
        if self.is_volume_valid(volume_in_ml) is False:
            return False

        if speed is not None:
            self.top_velocity = speed
        else:
            self.ensure_default_top_velocity()

        steps = self.volume_to_step(volume_in_ml)
        packet = self._protocol.forge_move_to_packet(steps)
        self.write_and_read_from_pump(packet)

        if wait:
            self.wait_till_ready()

        return True

    def go_to_max_volume(self, speed=None, wait=False):
        """
        Moves the pump to the maximum volume.

        Args:
            speed (int): The speed of movement, default set to None.

            wait (bool): Waits until the pump is idle, default set to False.

        Returns:
            True (bool): The maximum volume is valid.

            False (bool): The maximum volume is not valid.

        """
        self.go_to_volume(self.total_volume, speed=speed, wait=wait)

    def get_raw_valve_position(self):
        """
        Gets the raw value of the valve's position.

        Returns:
            raw_valve_position (str): The raw position of the valve.

        """
        valve_position_packet = self._protocol.forge_report_valve_position_packet()
        (_, raw_valve_position) = self.write_and_read_from_pump(valve_position_packet)
        return raw_valve_position

    def get_valve_position(self) -> str:
        """
        Gets the position of the valve.

        Returns:
            (str): The position of the valve.

        Raises:
            ValueError: The valve position is not valid/unknown.

        """
        raw_valve_position = self.get_raw_valve_position()
        if raw_valve_position == 'i':
            return VALVE_INPUT
        elif raw_valve_position == 'o':
            return VALVE_OUTPUT
        elif raw_valve_position == 'b':
            return VALVE_BYPASS
        elif raw_valve_position == 'e':
            return VALVE_EXTRA
        elif raw_valve_position in VALVE_6WAY_LIST:
            return raw_valve_position
        raise ValueError(f'Valve position received was {raw_valve_position}. It is unknown')

    def set_valve_position(self, valve_position, max_repeat=MAX_REPEAT_OPERATION, secure=True):
        """
        Sets the position of the valve.

        Args:
            valve_position (str): Position of the valve.

            max_repeat (int): maximum number of times to repeat an operation, default set to MAX_REPEAT_OPERATION (10).

            secure (bool): Ensures that everything is correct, default set to True.

        Returns:
            True (bool): The valve position has been set.

        Raises:
            ValueError: The valve position is invalid/unknown.

            ControllerRepeatedError: Too many failed attempts in set_valve_position.

        """
        for i in range(max_repeat):

            if self.get_valve_position() == valve_position:
                return True
            else:
                self.logger.debug("Valve not in position, change attempt {}/{}".format(i + 1, max_repeat))

            if valve_position == VALVE_INPUT:
                valve_position_packet = self._protocol.forge_valve_input_packet()
            elif valve_position == VALVE_OUTPUT:
                valve_position_packet = self._protocol.forge_valve_output_packet()
            elif valve_position == VALVE_BYPASS:
                valve_position_packet = self._protocol.forge_valve_bypass_packet()
            elif valve_position == VALVE_EXTRA:
                valve_position_packet = self._protocol.forge_valve_extra_packet()
            elif valve_position in VALVE_6WAY_LIST:
                valve_position_packet = self._protocol.forge_valve_6way_packet(valve_position)
            else:
                raise ValueError('Valve position {} unknown'.format(valve_position))

            self.write_and_read_from_pump(valve_position_packet)

            # if do not want to wait and check things went well, return now
            if secure is False:
                return True

            self.wait_till_ready()

        self.logger.debug(f"Too many failed attempts in set_valve_position for pump {self.name}!")
        raise ControllerRepeatedError(f"Repeated Error from pump {self.name}")

    def set_eeprom_config(self, operand_value: int):
        """
        Sets the configuration of the EEPROM on the pumps.

        Args:
            operand_value (int): The value of the supplied operand.

        """
        eeprom_config_packet = self._protocol.forge_eeprom_config_packet(operand_value)
        self.write_and_read_from_pump(eeprom_config_packet)

        eeprom_sign_packet = self._protocol.forge_eeprom_lowlevel_config_packet(sub_command=20, operand_value="pycont2")
        self.write_and_read_from_pump(eeprom_sign_packet)

        if operand_value == 1:
            print("####################################################")
            print("3-Way Y-Valve: Connect jumper to pin 5 (bottom pin) below address switch at back of pump")
            print("Unpower and repower the pump to activate changes!")
            print("####################################################")
        else:
            print("####################################################")
            print("Unpower and repower the pump to make changes active!")
            print("####################################################")

    def set_eeprom_lowlevel_config(self, command: int, operand: str):
        """
        Sets the configuration of the EEPROM on the pumps.

        Args:
            command (int): The value of the command to be issued.
            operand (int): The value of the supplied operand.

        """
        eeprom_packet = self._protocol.forge_eeprom_lowlevel_config_packet(sub_command=command, operand_value=operand)
        self.write_and_read_from_pump(eeprom_packet)

    def flash_eeprom_3_way_y_valve(self):
        """
        Sets the EEPROM config of the pump to use a 3-way Y valve (I/O operations)
        Requires switching of the jumper pin on the back of the pump from the top set of pins to the bottom.
        """
        self.set_eeprom_config(1)

    def flash_eeprom_3_way_t_valve(self):
        """
        Sets the EEPROM config of the pump to use a 3-way T valve (I/O operations)
        """
        self.set_eeprom_config(5)

    def flash_eeprom_4_way_nondist_valve(self):
        """
        Sets the EEPROM config of the pump to use a 4-way Non-Dist valve (I/O/E operations)
        Note in this configuration it is not possible to pump to E!
        valve position E connects E with O while B connects E and I (90-degrees)
        """
        self.set_eeprom_config(2)

    def flash_eeprom_4_way_dist_valve(self):
        """
        Sets the EEPROM config of the pump to use a 4-way Dist Valve (I/O/E operations)
        """
        self.set_eeprom_config(4)

    def get_eeprom_config(self):
        """
        Gets the EEPROM configuration.

        Returns:
            eeprom_config (int): The configuration of the EEPROM.

        """
        (_, eeprom_config) = self.write_and_read_from_pump(self._protocol.forge_report_eeprom_packet())
        return eeprom_config

    def get_current_valve_config(self):
        """
        Infers the current valve configuration based on the EEPROM data.
        """
        current_eeprom_config = self.get_eeprom_config().split(',')
        valve_config = current_eeprom_config[10]
        # Valve config: IOBEXYZ
        # [I]nput, [O]utput, [B]ypass, [E]xtra positions: n*90 deg (e.g. 0 -> 0 deg, 2 -> 180 deg)
        # [X], [Y] allow plunger movement in [B] and [E], respectively (Y=1 for DIST to enable delivering to E!)
        # [Z] swap the bypass and extra position on a 4-position valve if a [Y] initialization command is issued.

        if valve_config == "2013100":
            # flash_eeprom_3_way_t_valve() AND flash_eeprom_3_way_y_valve(). Difference is jumper J2-5, check with ?28
            current_valve_config = "3-WAY"
        elif valve_config == "2033110":
            # flash_eeprom_4_way_dist_valve()
            current_valve_config = "4-WAY dist"
        elif valve_config == "2130001":
            # flash_eeprom_4_way_nondist_valve()
            current_valve_config = "4-WAY nondist"
        else:
            # e.g. DEBUG:pycont.DTStatus:Received /0`10,75,14,62,1,1,20,10,48,210,2013010,0,0,0,0,0,25,20,15,0000000
            print(valve_config)
            current_valve_config = "Unknown"

        return current_valve_config

    def terminate(self):
        """
        Sends the command to terminate the current action.
        """
        self.write_and_read_from_pump(self._protocol.forge_terminate_packet())


class MultiPumpController(object):
    """
    This class deals with controlling multiple pumps on one or more hubs at a time.

    Args:
        setup_config (Dict): The configuration of the setup.

    """
    def __init__(self, setup_config):
        self.logger = create_logger(self.__class__.__name__)
        self.pumps = {}

        # Sets groups and default configs if provided in the config dictionary
        self.groups = setup_config['groups'] if 'groups' in setup_config else {}
        self.default_config = setup_config['default'] if 'default' in setup_config else {}

        for pump_name, pump_config in list(setup_config['pumps'].items()):
            full_pump_config = self.default_pump_config(pump_config)
            self.pumps[pump_name] = C3000Controller.from_config(pump_name, full_pump_config)

        # Adds pumps as attributes
        self.set_pumps_as_attributes()

    @classmethod
    def from_configfile(cls, setup_configfile):
        """
        Obtains the configuration data from the supplied configuration file.

        Args:
            cls (Class): The initialising class.

            setup_configfile (File): The configuration file.

        Returns:
            MultiPumpController: A new MultiPumpController object with the configuration set from the config file.

        """
        with open(setup_configfile) as f:
            return cls(json.load(f))

    def default_pump_config(self, pump_specific_config):
        """
        Creates a default pump configuration.

        Args:
            pump_specific_config (Dict): Dictionary containing the pump configuration.

        Returns:
            combined_pump_config (Dict): A new default pump configuration mirroring that of pump_config.

        """
        # Makes a copy of the default values (this is needed because we are going to merge default with pump settings)
        combined_pump_config = dict(self.default_config)

        # Adds pump specific settings
        for k, v in list(pump_specific_config.items()):
            combined_pump_config[k] = v

        # Returns the combination of default settings and pump specific settings
        return combined_pump_config

    def set_pumps_as_attributes(self):
        """ Sets the pumps as attributes. """
        for pump_name, pump in list(self.pumps.items()):
            if hasattr(self, pump_name):
                self.logger.warning(f"Pump named {pump_name} is a reserved attribute, please change name or do not use "
                                    f"this pump in attribute mode, rather use pumps['{pump_name}'']")
            else:
                setattr(self, pump_name, pump)

    def get_pumps(self, pump_names) -> List[C3000Controller]:
        """
        Obtains a list of all pumps with name in pump_names.

        Args:
            pump_names (List): A list of the pump names

        Returns:
            pumps (List): A list of the pump objects.

        """
        pumps = []
        for pump_name in pump_names:
            try:
                pumps.append(self.pumps[pump_name])
            except KeyError:
                pass
        return pumps

    def get_all_pumps(self) -> List[C3000Controller]:
        """ Obtains a list of all pumps. """
        return list(self.pumps.values())

    def get_pumps_in_group(self, group_name):
        """
        Obtains a list of all pumps with group_name.

        Args:
            group_name (List): The group name

        Returns:
            pumps (List): A list of the pump objects in the group. None for non-existing groups.

        """
        pumps = []
        try:
            pump_list = self.groups[group_name]
        except KeyError:
            return None

        for pump_name in pump_list:
            pumps.append(self.pumps[pump_name])
        return pumps

    def apply_command_to_pumps(self, pump_names, command, *args, **kwargs):
        """
                Applies a given command to the pumps.

                Args:
                    pump_names (List): List containing the pump names.

                    command (str): The command to apply.

                    *args: Variable length argument list.

                    **kwargs: Arbitrary keyword arguments.

                Returns:
                    returns (Dict): Dictionary of the functions return values.

                """
        returns = {}
        for pump_name in pump_names:
            func = getattr(self.pumps[pump_name], command)
            returns[pump_name] = func(*args, **kwargs)

        return returns

    def apply_command_to_all_pumps(self, command, *args, **kwargs):
        """
        Applies a given command to all of the pumps.

        Args:
            command (str): The command to apply.

            *args: Variable length argument list.

            **kwargs: Arbitrary keyword arguments.

        Returns:
            returns (Dict): Dictionary of the functions.

        """
        return self.apply_command_to_pumps(list(self.pumps.keys()), command, *args, **kwargs)

    def apply_command_to_group(self, group_name, command, *args, **kwargs):
        """
        Applies a given command to the group.

        Args:
            group_name (str): Name of the group.

            command (str): The command to apply.

            *args: Variable length argument list.

            **kwargs: Arbitrary keyword arguments.

        Returns:
            returns (Dict) Dictionary of the functions.

        """
        return self.apply_command_to_pumps(self.groups[group_name], command, *args, **kwargs)

    def are_pumps_initialized(self):
        """
        Determines if the pumps have been initialised.

        Returns:
            True (bool): The pumps have been initialised.

            False (bool): The pumps have not been initialised.

        """
        for pump in self.pumps.values():
            if not pump.is_initialized():
                return False
        return True

    def smart_initialize(self, secure=True):
        """
        Initialises the pumps, setting all parameters.

        Args:
            secure (bool): Ensures everything is correct, default set to True.

        """
        for pump in self.pumps.values():
            if not pump.is_initialized():
                pump.initialize_valve_only(wait=False)
        self.wait_until_all_pumps_idle()

        for pump in self.pumps.values():
            if not pump.is_initialized():
                pump.set_valve_position(pump.initialize_valve_position, secure=secure)
        self.wait_until_all_pumps_idle()

        for pump in self.pumps.values():
            if not pump.is_initialized():
                pump.initialize_no_valve(wait=False)
        self.wait_until_all_pumps_idle()

        for pump in self.pumps.values():
            if not pump.is_initialized():
                pump.check_jumper_position()
        self.wait_until_all_pumps_idle()

        self.apply_command_to_all_pumps('init_all_pump_parameters', secure=secure)
        self.wait_until_all_pumps_idle()

    def wait_until_all_pumps_idle(self):
        """
        Sends the command 'wait_till_ready' to the pumps.
        """
        self.apply_command_to_all_pumps('wait_till_ready')

    def wait_until_group_idle(self, group_name):
        """
        Sends the command ' wait_till_ready' to all pumps of a group.
        """
        self.apply_command_to_group(group_name=group_name, command='wait_till_ready')

    def terminate_all_pumps(self):
        """
        Sends the command 'terminate' to all the pumps.
        """
        self.apply_command_to_all_pumps('terminate')

    def pump(self, pump_names, volume_in_ml, from_valve=None, speed_in=None, wait=False, secure=True):
        """
        Pumps the desired volume.

        Args:
            pump_names (List): The name of the pumps.

            volume_in_ml (float): The volume to be pumped.

            from_valve (chr): The valve to pump from.

            speed_in (int): The speed at which to pump, default set to None.

            wait (bool): Waits for the pumps to be idle, default set to False.

            secure (bool): Ensures everything is correct, default set to False.

        """
        if speed_in is not None:
            self.apply_command_to_pumps(pump_names, 'set_top_velocity', speed_in, secure=secure)
        else:
            self.apply_command_to_pumps(pump_names, 'ensure_default_top_velocity', secure=secure)

        if from_valve is not None:
            self.apply_command_to_pumps(pump_names, 'set_valve_position', from_valve, secure=secure)

        self.apply_command_to_pumps(pump_names, 'pump', volume_in_ml, speed_in=speed_in, wait=False)

        if wait:
            self.apply_command_to_pumps(pump_names, 'wait_till_ready')

    def deliver(self, pump_names, volume_in_ml, to_valve=None, speed_out=None, wait=False, secure=True):
        """
        Delivers the desired volume.

        Args:
            pump_names (List): The name of the pumps.

            volume_in_ml (float): The volume to be delivered.

            to_valve (chr): The valve to deliver to.

            speed_out (int): The speed at which to deliver.

            wait (bool): Wait for the pumps to be idle ,default set to False.

            secure (bool): Ensures everything is correct, default set to True.

        """
        if speed_out is not None:
            self.apply_command_to_pumps(pump_names, 'set_top_velocity', speed_out, secure=secure)
        else:
            self.apply_command_to_pumps(pump_names, 'ensure_default_top_velocity', secure=secure)

        if to_valve is not None:
            self.apply_command_to_pumps(pump_names, 'set_valve_position', to_valve, secure=secure)

        self.apply_command_to_pumps(pump_names, 'deliver', volume_in_ml, speed_out=speed_out, wait=False)

        if wait:
            self.apply_command_to_pumps(pump_names, 'wait_till_ready')

    def transfer(self, pump_names, volume_in_ml, from_valve, to_valve, speed_in=None, speed_out=None, secure=True):
        """
        Transfers the desired volume between pumps.

        Args:
            pump_names (List): The name of the pumps.

            volume_in_ml (float): The volume to be transferred.

            from_valve (chr): The valve to transfer from.

            to_valve (chr): the valve to transfer to.

            speed_in (int): The speed at which to receive transfer, default set to None.

            speed_out (int): The speed at which to transfer, default set to None

            secure (bool): Ensures that everything is correct, default set to False.

        """
        volume_transferred = float('inf')  # Temporary value for the first cycle only, see below
        for pump in self.get_pumps(pump_names):
            candidate_volume = min(volume_in_ml, pump.remaining_volume)  # Smallest target and remaining is candidate
            volume_transferred = min(candidate_volume, volume_transferred)  # Transferred is global minimum

        self.pump(pump_names, volume_transferred, from_valve, speed_in=speed_in, wait=True, secure=secure)
        self.deliver(pump_names, volume_transferred, to_valve, speed_out=speed_out, wait=True, secure=secure)

        remaining_volume_to_transfer = volume_in_ml - volume_transferred
        if remaining_volume_to_transfer > 0:
            self.transfer(pump_names, remaining_volume_to_transfer, from_valve, to_valve, speed_in, speed_out)
