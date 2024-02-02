try:
#   print("starting up")
    import gc
    import select
    import sys
    import machine
    import time
    import _thread
    import hx711
#   print(f"Free Memory: {gc.mem_free()} bytes")

    FIRMWARE = "Aeon Laboratories LN Scale"
    VERSION = "20240110-0000"

    # how often to run garbage collection
    GC_PERIOD = 2000 # every 2 seconds
    
    # Hz, maximum commands per second (minimum command pacing: ~20 ms)
    POLL_FREQ = 50
    
    # microseconds to wait for a command (at 0, we're really just checking if one has already been received)
    POLL_TIMEOUT = 0

    # Hz, how fast to flash the Pico LED to indicate the program is running.
    FLASH_FREQ = 3

    # gpio pins for HX711 bridge amplifiers
    HX1_CLOCK = 27
    HX1_DATA = 26
    HX2_CLOCK = 7
    HX2_DATA = 6
    
    def discard_timer(timer):
        if timer is None:
            return
        timer.deinit()
        timer = None

    def flash_led(timer=None):
        pico_led.on()
        time.sleep_ms(5)
        pico_led.off()

    def detect_input():
        """
        Check stdin for new input. This method cannot be trusted if
        poller is tracking multiple streams.
        """
        try:
            i = next(poller.ipoll(POLL_TIMEOUT))
            return (i[0] == sys.stdin and i[1] == select.POLLIN)
        except StopIteration:
            return False

    def getch():
        return sys.stdin.read(1)        

    def get_input(timer=None):
        """
        Do nothing unless the global variable input_text is free (empty).
        Check stdin for data. If new input has been received, store it in input_text
        and trigger the command processor.

        Args:
            timer (machine.Timer): Allows get_input to function as a timer callback. This
            value is not used within the function.
        """
        global input_text
        
        # Don't check for input unless the previous command has already been consumed for processing.
        # get_command() clears input_text after copying the command
        if input_text:
            return
        
        if detect_input():
            # wait for the entire command. 20 characters take about 1.8 ms at 115200 baud.
            time.sleep_ms(3)
            
            # Because python lists are mutable and strings are not, 
            # append + join should become faster than repeated concatenation
            # at some point due to fewer memory allocations. I don't know
            # where the crossover occurs, but I imagine that any added expense
            # of the list approach should be negligible at the low end.
            buf = [getch()] # buf is a list
            while detect_input():
                buf.append(getch())
            buf = ''.join(buf) # now buf is a string
            buf = buf.strip();
            if buf:
#               print(f"{time.ticks_us()} input_text = '{input_text}': {len(input_text)}")
                with input_lock:
                    input_text = buf
                machine.Timer(period=0, mode=machine.Timer.ONE_SHOT, callback=process_command)

    def gc_collect(timer=None):
        pre = gc.mem_free()
        gc.collect()    # typically consumes about 2 milliseconds
        global gc_collected
        gc_collected = gc.mem_free() - pre
        
    def tolerant_float(s):
        try:
            return float(s)
        except ValueError:
            return 0

    def get_command():
        global input_text
        cmd = input_text
        with input_lock:
            input_text = ''    # clearing input_text enables reading another
        return cmd

    # a command string is a command keyword followed by a list of numbers,
    # all separated by whitespace
    def parse_command_string(command_string):
        tokens = command_string.split()
        if not tokens:
            return "", []

        command = tokens[0]
        if len(tokens) == 1:
            return command, []
        
        return command, [tolerant_float(token) for token in tokens[1:]]

    def report(timer=None):
        bridge1 = hx1.value
        bridge2 = hx2.value
        both = bridge1+bridge2
        print(f"{both: 8.2f} {bridge1: 8.2f} {bridge2: 8.2f}")
        
    def process_command(timer):
        command, numbers = parse_command_string(get_command())
#       print(f"'{command}' {numbers} Free Memory: {gc.mem_free()} bytes ({gc_collected} collected)")
        if command:
#           start = time.ticks_us()
            if command == 'r':
                if len(numbers) == 0:
                    report()
                else:
                    global report_timer
                    discard_timer(report_timer)
                    p = min(max(round(numbers[0]), 0), 300) # 0 to 300 seconds (0-5 min)
                    if p > 0:
                        p *= 1000 # milliseconds
                        report_timer = machine.Timer(period=p, callback=report)
                
            elif command == '0':
                hx1.zero_now()
                hx2.zero_now()
            elif command == 'g':    # set gains, accepts up to two numbers
                nn = len(numbers)
                if nn > 0:
                    hx1.gain = numbers[0]
                    hx2.gain = numbers[1] if nn > 1 else numbers[0]                    
            elif command == 'z':
                print(FIRMWARE)
                print(VERSION)
                print(f"Free Memory: {gc.mem_free()} bytes")
            elif command == 'shutdown':
                global running
                running = False
#           print(f"command processing time: {time.ticks_diff(time.ticks_us(), start)} us")
            discard_timer(timer)

#==================================================================================#
# main code (scoped here at the module level for trivial access to these globals)
    running = True
    gc_collected = 0
    gc_timer = machine.Timer(period=GC_PERIOD, callback=gc_collect)
    
    hx1 = hx711.Hx711(HX1_CLOCK, HX1_DATA)
    hx2 = hx711.Hx711(HX2_CLOCK, HX2_DATA)
    
    pico_led = machine.Pin("LED", machine.Pin.OUT)
    flash_timer = machine.Timer(freq=FLASH_FREQ, callback=flash_led)
    
    input_text = ''
    input_lock = _thread.allocate_lock()    
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    report_timer = None
    poll_timer = machine.Timer(freq=POLL_FREQ, callback=get_input)

    # idle loop
    while running:
        time.sleep_ms(1000)

    discard_timer(report_timer)
    discard_timer(poll_timer)
    discard_timer(gc_timer)
    discard_timer(flash_timer)
    hx1.power_down()
    hx2.power_down()
    hx1 = None
    hx2 = None
    gc_collect()
    print(f"Free Memory: {gc.mem_free()} bytes.\r\nShutdown complete.")
#     machine.soft_reset()
#==================================================================================#

except (KeyboardInterrupt):
    # On Ctrl-C aka ASCII ETX (0x03)
    machine.reset()
