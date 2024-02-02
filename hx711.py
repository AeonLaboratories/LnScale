"""
This module enables data acquisition from an HX711 bridge-ADC circuit connected to the Raspberry Pi Pico.

Usage:
- Connect the HX711 to the appropriate GPIO pins on the Pico.
- Create an instance of the Hx711 class for each HX711 module you want to interface with.
- Use the methods of the Hx711 class to read data, set gain, and perform other operations.

Attributes:
    UPDATE_FREQ (int): Nominal ADC cycle time when RATE pin is held low.
    GAIN_128 (int): Configuration for Channel A, gain 128
    GAIN_32 (int): Configuration for Channel B, gain 32.
    GAIN_64 (int): Configuration for Channel A, gain 64.
    instance_list (list): List of connected HX711 modules.
    read_timer (machine.Timer): Timer for periodic reading of all connected HX711 modules.
    SETTLING_TIME (int): Milliseconds needed after power up, reset, or configuration change.
    DATA_READY_TIMEOUT (int): Timeout for data ready indication (DOUT signal == 0).
"""
import sys
import _thread
import time
import machine
from micropython import const

DEBUG_ELAPSED = -1
# start = time.ticks_us()
# global DEBUG_ELAPSED
# DEBUG_ELAPSED = time.ticks_diff(time.ticks_us(), start)


# Status flags
STATUS_NOMINAL = const(0)               # Not a flag; the value of status when no flags are set
STATUS_INITIALIZING = const(1)
STATUS_POWERED_DOWN = const(2)
STATUS_POWERING_UP = const(4)
STATUS_NOT_SETTLED = const(8)
STATUS_DATA_NOT_READY = const(16)
STATUS_DATA_READY_TIMEOUT = const(32)
STATUS_NO_DATA = const(64)
STATUS_DOUT_STUCK_LOW = const(128)
STATUS_ZEROING = const(256)
STATUS_OUT_OF_RANGE = const(512)

# Status descriptions
STATUS_DESCRIPTIONS = {
    STATUS_NOMINAL: "No ususual status conditions are present",
    STATUS_INITIALIZING: "The instance is being initialized",
    STATUS_POWERED_DOWN: "The HX711 is powered down",
    STATUS_POWERING_UP: "The HX711 is powering up",
    STATUS_NOT_SETTLED: "The HX711 hasn't had time to settle since the last power up or configuration change",
    STATUS_DATA_NOT_READY: "Data is not ready",
    STATUS_DATA_READY_TIMEOUT: "Data was not ready for too long",
    STATUS_NO_DATA: "Data wasn't available on the last attempted read",
    STATUS_DOUT_STUCK_LOW: "The data pin failed to go low on the 25th clock pulse",
    STATUS_ZEROING: "Collecting readings to determine the zero offset",
    STATUS_OUT_OF_RANGE: "The HX711 output is at its minimum or maximum possible value"
}

# The nominal ADC cycle frequency when the HX711 RATE
# pin is held low (RATE=0, 10 samples per second).
# TODO: extend this to include the RATE=1, 80 samples-per-second option.
UPDATE_FREQ = const(10)

MAX_RAW: int = const(0x7fffff)       # maximum 24-bit int (the HX711 outputs 24 bits)
MIN_RAW: int = const(-0x800000)
EXTEND_SIGN: int = const(0x1000000)  # subtract this from a 24-bit value to extend the sign

GAIN_128: int = const(0)  # Configuration for Channel A, gain 128
GAIN_64: int = const(2)   # Configuration for Channel A, gain 64
GAIN_32: int = const(1)   # Configuration for Channel B, gain 32

# The settling time in milliseconds needed after a power up, reset,
# or configuration change, when the sample rate is 10 per second
# (HX711 RATE pin = 0).
# TODO: extend this to include the 80 samples-per-second option.
SETTLING_TIME = const(400)

# When the HX711 completes an AD conversion, it sets its DOUT signal low,
# indicating that it has data ready for retrieval. This module's
# UPDATE_FREQ is intended to always allow sufficient time for the
# HX711 to complete an ADC conversion between updates. So, DOUT
# should always be low when read_timer ticks.
# DATA_READY_TIMEOUT is the maximum time this module allows for the
# DOUT signal to remain high before it sets STATUS_DATA_READY_TIMEOUT.
DATA_READY_TIMEOUT = round(2000 / UPDATE_FREQ)    # milliseconds

def validate_gain(gain):
    """
    Ensure the gain value is one of the allowed values,
    hx711.GAIN_128, hx711.GAIN_64, or hx711.GAIN_32.

    Raises:
        ValueError: If an invalid gain value is provided.
    """
    if not (gain == GAIN_128 or gain == GAIN_64 or gain == GAIN_32):
        raise ValueError(f"Invalid gain value: {gain}. Must be hx711.GAIN_128, hx711.GAIN_64, or hx711.GAIN_32")

def discard_timer(timer):
    """
    Discard the provided timer.

    Args:
        timer (machine.Timer): Timer to be discarded.
    """
    if timer is None:
        return
    timer.deinit()
    timer = None

def read_all(timer):
    """
    Retrieve a new value from each connected HX711.

    Args:
        timer (machine.Timer): Timer triggering the reading.
    """
    
    # It takes about 3.4 milliseconds to complete this loop with two instances
    for item in instance_list:
        item.read()
    
instance_list = []
read_timer = machine.Timer(mode=machine.Timer.PERIODIC, freq=UPDATE_FREQ, callback=read_all)

class Hx711:
    """
    Hx711 class for interacting with the HX711 bridge-ADC module.

    Attributes:
        name (str): Identifier for this instance.
        status (int): Current status flags indicating the state of the Hx711 instance.
        clock_pin_no (int): GPIO pin number connected to HX711 signal PD_SCK ("Power Down, Signal Clock").
        data_pin_no (int): GPIO pin number connected to HX711 signal DOUT ("Data OUTput").
        configuration (int): Selects the HX711 PGIA gain and input channel; 128 and A by default; use the GAIN_ constants.
        offset (int): Offset correction to be subtracted from the HX711 ADC output.
        gain (float): Multiplier to convert the offset-corrected HX711 output count to meaningful units.
        zeros_to_average (int): Number of consecutive read values to use when calculating the default self.offset.
        value (float): The adjusted result: value = (HX711 data output - offset) * gain.
        
    Methods:
        __init__(self, clock_pin_no, data_pin_no, gain=GAIN_128):
            Initialize an Hx711 instance.

            Args:
                clock_pin_no (int): machine.Pin for HX711 signal PD_SCK ("Power Down, Signal Clock").
                data_pin_no (int): machine.Pin for HX711 signal DOUT ("Data OUTput").
                gain (int): Gain/channel configuration (default: GAIN_128).

            Raises:
                ValueError: If an invalid gain value is provided.

        __del__(self):
            Destructor. Power down and release the timers when the instance is deleted.

        clear_status(self, flags):
            Clear the given status flags.

            Args:
                flags (int): Status flags to clear.

        set_status(self, flags):
            Set the given status flags.

            Args:
                flags (int): Status flags to set.

        status_contains(self, flags):
            Check if any of the specified status flags are set.

            Args:
                flags (int): Status flags to check.

            Returns:
                bool: True if any of the status flags are set, or if flags is STATUS_NOMINAL and none are set; False otherwise.

        set_gain(self, gain=GAIN_128):
            Ensure the specified HX711 gain / channel is configured.

            Args:
                gain (int): Gain/channel configuration (default: hx711.GAIN_128).

            Returns:
                bool: True if a change was made, False otherwise.

            Raises:
                ValueError: If an invalid gain value is provided.

        power_down(self):
            Power down the HX711.

        power_up(self):
            Powers up the HX711.

        powered_up(self):
            Check if the HX711 is powered up.

            Returns:
                bool: True if powered up, False otherwise.

        reset(self):
            Reset the HX711: power down and back up.

        settled_timeout(self, timer):
            Callback for the settled timer. Discard the timer when it elapses.

            Args:
                timer (machine.Timer): Timer triggering the callback.

        settled(self):
            Check if the HX711 is powered up and has had time to settle since the last
            reset or configuration change.

            Returns:
                bool: True if settled, False otherwise.

        wait_settled(self):
            Wait only if powered up and not yet settled.

        data_ready_timeout(self, timer):
            Callback for the data ready timeout timer.

            Args:
                timer (machine.Timer): Timer triggering the callback.

        data_ready(self):
            Check if the HX711 data is ready. If its not, start a data ready timeout timer.
            Whenever the data is ready, discard the timer, so it doesn't trigger the timeout status.

            Returns:
                bool: True if data is ready, False otherwise.

        wait_data_ready(self, timeout=DATA_READY_TIMEOUT):
            Wait up to timeout milliseconds for data ready (HX711 ADC conversion complete).

            Args:
                timeout (int): Timeout duration in milliseconds.

            Returns:
                bool: True if data is ready within the specified timeout, False otherwise.

        clock_a_data_bit(self):
            Send a clock pulse and return the state of the data line.

            Returns:
                int: State of the data line (1 or 0).

        read(self):
            Acquire the ADC count from the HX711 and save the adjusted result in
            the Hx711 value attribute.

            Returns:
                float: gain * (ADC count - offset)

        zero_now(self):
            Begin averaging readings to find the HX711 zero.
    """
    def __init__(self, clock_pin_no, data_pin_no, gain=GAIN_128):
        """
        Initialize an Hx711 instance.

        Args:
            clock_pin_no (int): machine.Pin for HX711 signal PD_SCK ("Power Down, Signal Clock").
            data_pin_no (int): machine.Pin for HX711 signal DOUT ("Data OUTput").
            gain (int): Gain/channel configuration (default: GAIN_128).

        Raises:
            ValueError: If an invalid gain value is provided.
        """
        #It takes about 2 ms to initialize an instance
        self.status = STATUS_INITIALIZING
        validate_gain(gain)
        self.name = f"Hx711({clock_pin_no},{data_pin_no}): "
        self.lock = _thread.allocate_lock()
        self.clock_pin_no = clock_pin_no  # HX711 signal PD_SCK ("Power Down, Signal ClocK")
        self.clock = machine.Pin(clock_pin_no, machine.Pin.OUT)
        self.data_pin_no = data_pin_no  # HX711 signal DOUT ("Data OUTput")
        self.data = machine.Pin(data_pin_no, machine.Pin.IN)
        self.configuration = GAIN_128  # input A, gain 128 (HX711 default)
        self.data_ready_timer = None
        self.settled_timer = None
        self.offset = 0
        self.gain = 1.0
        self.dfs = 0.0    # digital filter stablity; see weighted_average()
        self.zeros_to_average = 50    # about 5 seconds
        self.value = 0
        self.reset()
        instance_list.append(self)
        self.clear_status(STATUS_INITIALIZING)

    def __del__(self):
        """
        Destructor. Power down and release timers when the instance is deleted.
        """
        instance_list.remove(self)
        self.power_down()
        discard_timer(self.data_ready_timer)
        discard_timer(self.settled_timer)

    def clear_status(self, flags):
        """
        Clear the given status flags.

        Args:
            flags (int): Status flags to clear.
        """
        self.status &= ~flags
        
    def set_status(self, flags):
        """
        Set the given status flags.

        Args:
            flags (int): Status flags to set.
        """
        self.status |= flags

    def status_contains(self, flags):
        """
        Check if any of the specified status flags are set.

        Args:
            flags (int): Status flags to check.

        Returns:
            bool: True if any of the status flags are set, or if flags is STATUS_NOMINAL and none are set; False otherwise.
        """
        if (flags == STATUS_NOMINAL):
            return self.status == STATUS_NOMINAL
        return bool(self.status & flags)

    def set_gain(self, gain=GAIN_128):
        """
        Ensure the desired gain / channel are selected.

        Args:
            gain (int): Gain/channel configuration (default: GAIN_128).

        Returns:
            bool: True if a change was made, False otherwise.

        Raises:
            ValueError: If an invalid gain value is provided.
        """
        validate_gain(gain)

        if gain != self.configuration:
            self.configuration = gain
            self.wait_data_ready()
            self.read()  # select the input and set the gain
            self.set_status(STATUS_NOT_SETTLED)
            discard_timer(self.settled_timer)
            self.settled_timer = machine.Timer(period=SETTLING_TIME, mode=machine.Timer.ONE_SHOT, callback=self.settled_timeout)
            return True
        
        return False

    def power_down(self):
        """
        Power down the HX711.
        The HX711 powers down when clock is held high for more than 60 us.
        It stays off as long as clock remains high, and powers up when
        clock goes low.        
        """
        # Seting the status at the beginning is slightly premature, but correct
        # on the presumption that anything checking would prefer this meaning.
        self.set_status(STATUS_POWERED_DOWN | STATUS_NOT_SETTLED)   # | meaning both
        self.clock(True)
        time.sleep_us(60)

    def power_up(self):
        """
        HX711 power is enabled when the clock signal is low
        and stays enabled unless Clock goes high for > 60 us.
        Thus, short clock pulses (normally 1 us) don't power 
        down the HX711.
        """
        self.set_status(STATUS_POWERING_UP | STATUS_NOT_SETTLED)
        self.zero_now()
        self.clock(False)  # HX711 power-up signal
        self.clear_status(STATUS_POWERED_DOWN)
        discard_timer(self.settled_timer)
        self.settled_timer = machine.Timer(period=SETTLING_TIME, mode=machine.Timer.ONE_SHOT, callback=self.settled_timeout)
        self.set_gain()
        self.clear_status(STATUS_POWERING_UP)
        
    def powered_up(self):
        return not self.status_contains(STATUS_INITIALIZING | STATUS_POWERED_DOWN | STATUS_POWERING_UP)

    def reset(self):
        """
        Reset the HX711: power down and back up.
        """
        self.power_down()
        self.power_up()

    def settled_timeout(self, timer):
        """
        Callback for the settled timer. Discard the timer when it elapses.

        Args:
            timer: machine.Timer triggering the callback.
        """
        self.clear_status(STATUS_NOT_SETTLED)
        discard_timer(self.settled_timer)

    def settled(self):
        """
        Check if the HX711 is powered up and has had time to settle since the last 
        reset or configuration change.

        Returns:
            bool: True if settled, False otherwise.
        """
        return self.powered_up() and not self.status_contains(STATUS_NOT_SETTLED)

    def wait_settled(self):
        """
        Wait only if powered up and not yet settled.
        """
        while self.powered_up() and not self.settled():
            time.sleep_ms(1)

    def data_ready_timeout(self, timer):
        """
        Callback for the data ready timeout timer.

        Args:
            timer: machine.Timer triggering the callback.
        """
        self.set_status(STATUS_DATA_READY_TIMEOUT)
        discard_timer(self.data_ready_timer)

    # HX711 sets DOUT low when AD conversion is complete / data is ready
    def data_ready(self):
        """
        Check if the HX711 data is ready.
        If its not, start a data ready timeout timer.
        Whenever the data is ready, discard the timer, so it doesn't trigger the timeout status.

        Returns:
            bool: True if data is ready, False otherwise.
        """
        ready = self.data() == 0  # AD conversion complete
        if ready:
            discard_timer(self.data_ready_timer)
            self.clear_status(STATUS_DATA_NOT_READY | STATUS_DATA_READY_TIMEOUT)
        else:
            self.set_status(STATUS_DATA_NOT_READY)
            if self.settled() and self.data_ready_timer is None:
                self.data_ready_timer = machine.Timer(period=DATA_READY_TIMEOUT, mode=machine.Timer.ONE_SHOT, callback=self.data_ready_timeout)
        return ready

    def wait_data_ready(self, timeout=DATA_READY_TIMEOUT):
        """
        Wait up to timeout milliseconds for data ready (HX711 ADC conversion complete).

        Args:
            timeout (int): Timeout duration in milliseconds.

        Returns:
            bool: True if data is ready within the specified timeout, False otherwise.
        """
        for ms in range(timeout):
            if self.data_ready():
                return True
            time.sleep_ms(1)
        return False

    def clock_a_data_bit(self):
        """
        Send a clock pulse and return the state of the data line.

        Returns:
            int: State of the data line (1 or 0).
        """
        self.clock(True)
        time.sleep_us(1)  # pulse time should be 0.2 to 50 microseconds (1 us is nominal)
        self.clock(False)
        # no need to wait here; data is already valid 0.1 us after clock goes high
        return self.data()

    def weighted_average(self, value):
        return self.value * self.dfs + value * (1 - self.dfs)

    def read(self):
        """
        Attempt to read the ADC count from the HX711 and save the adjusted result in
        the Hx711 value attribute. If new data is not ready or an error is 
        encountered, return the previous valid result.

        Returns:
            float: gain * (ADC count - offset)
        """
        # It takes about 1.5 ms to complete a normal reading without weighted averaging
        # and about 1.7 or 1.8 ms with it.
        
        if self.data_ready():
            self.clear_status(STATUS_NO_DATA)
        else:
            self.set_status(STATUS_NO_DATA)
            return self.value
        
        with self.lock:
            # Shift in the 24 data bits and set the gain.
            data = 0
            for j in range(24):
                data <<= 1
                data |= self.clock_a_data_bit()

            # The total number of clocks selects the input and sets the ADC gain:
            #     25 clocks selects input A, gain = 128
            #     26 clocks selects input B, gain = 32
            #     27 clocks selects input A, gain = 64.
            # A minimum of 25 clocks are required, and the HX711 should always
            # set the Data signal high after the 25th clock is sent.

            # The 25th clock:
            if self.clock_a_data_bit() == 1:
                self.clear_status(STATUS_DOUT_STUCK_LOW)
            else:
                self.set_status(STATUS_DOUT_STUCK_LOW)
                return self.value

            # Additional clocks to finish selecting or confirming the HX711 
            # gain/channel configuration.
            # This must be done after every data read.
            for j in range(self.configuration):
                self.clock_a_data_bit()

            #print(f"{self.name} raw: 0x{data:08x} = {data}")
            if data > MAX_RAW:          # it's negative
                data -= EXTEND_SIGN     # extend the sign
            if data == MAX_RAW or data == MIN_RAW:
                self.set_status(STATUS_OUT_OF_RANGE)
            else:
                self.clear_status(STATUS_OUT_OF_RANGE)
            #print(f"{self.name} data = {data}")

            # only increment values_received if the HX711 is settled
            if self.settled():
                # first handle zeroing if necessary
                if self.values_received < self.zeros_to_average:
                    self.zeroing_sum += data
                elif self.status_contains(STATUS_ZEROING) and self.values_received >= self.zeros_to_average:
                    if self.zeros_to_average > 0:
                        self.offset = round(self.zeroing_sum / self.zeros_to_average)
                    self.clear_status(STATUS_ZEROING)
                elif self.values_received == sys.maxsize:
                    # avoid re-zeroing if the counter ever rolls over
                    self.values_received = self.zeros_to_average                    

                self.values_received += 1

                if self.status == STATUS_NOMINAL:
                    data = self.gain * (data - self.offset)
                    
                    if self.dfs > 0 and self.dfs < 1 and self.values_received > self.zeros_to_average:
                        data = self.weighted_average(data)
                        
                    self.value = data
            
    def zero_now(self):
        """
        Begin averaging readings to find the HX711 zero.
        """
        self.zeroing_sum = 0;
        self.set_status(STATUS_ZEROING);
        self.values_received = 0
