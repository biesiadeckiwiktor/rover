pattern_running = False

def run_test_pattern(self_drive):
    global pattern_running
    pattern_running = True
    
    while pattern_running:
        if not pattern_running:
            break
        self_drive.make_specific_turn(150, 1, "left", 1)
        if not pattern_running:
            break
        self_drive.make_specific_turn(150, 1, "right", 1)
    
    self_drive.send("MOTORS:0,0,0,0,0,0")

def stop_pattern():
    global pattern_running
    pattern_running = False

