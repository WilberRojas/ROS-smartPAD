#!/usr/bin/env python3
from evdev import InputDevice, ecodes

# check event number in ~/dev/input --> ll | grep event
ps5_controller = InputDevice('/dev/input/event25')
print(ps5_controller)

buttons = {
    304: "X",
    305: "Circulo",
    307: "Triangulo",
    308: "Cuadrado",

    310: "L1",
    311: "R1",
    312: "L2",
    313: "R2",

    314: "Share",
    315: "Menu ≡",
    316: "PS5 Logo",
    317: "Boton Joystick Izquierdo",  
    318: "Boton Joystick Derecho"
}

buttons_val = {
    0: "Soltado",
    1: "Presionado"
}

absolutes = {                               # ecodes.EV_ABS
    0: 'left joystick left/right',          # 0 = left, 255 = right
    1: 'left joystick up/down',             # 0 = up, 255 = down
    4: 'right joystick left/right',         # 0 = left, 255 = right
    2: 'L2 analog',                         # 0 = no press, 255 = full press
    5: 'R2 analog',                         # 0 = no press, 255 = full press
    3: 'right joystick up/down',            # 0 = up, 255 = down
    16: 'leftpad left/right',               # -1 = left, 0 = stop pressing, 1 = right
    17: 'leftpad up/down',                  # -1 = up, 0 = stop pressing, 1 = down
}

leftpad_left_right_values = {
    -1: 'left',
    0: 'left-right stop',                   # stoip means that the button was no longer pressed
    1: 'right'
}

leftpad_up_down_values = {
    -1: 'up',
    0: 'up-down stop',
    1: 'down'
}

CENTER = 128
BLIND = 20                                   # there's a lot of drift at 128, so don't report changes within (128 - this value)

left_joystick, right_joystick = [CENTER, CENTER], [CENTER, CENTER]

def update_left_joystick_position(event):
    global left_joystick
    if event.code == 0:                     # left joystick, x-axis (left/right)
        left_joystick[0] = value
    elif event.code == 1:                   # left joystick, y-axis (up/down)
        left_joystick[1] = 255-value

def update_right_joystick_position(event):
    global right_joystick
    if event.code == 3:                     # right joystick, x-axis (left/right)
        right_joystick[0] = value
    elif event.code == 4:                   # right joystick, y-axis (up/down)
        right_joystick[1] = 255-value

def decode_leftpad(event):
    action  = ''
    if event.code == 16:                    # leftpad, either a left or right action
        action = leftpad_left_right_values[value]
    elif event.code == 17:                  # leftpad, either an up or down action
        action = leftpad_up_down_values[value]
    else:                                   # unhandled event
        return ''
    return f'leftpad: {action}'



for event in ps5_controller.read_loop():    
    if event.type == ecodes.EV_KEY and event.code in buttons:
        button = buttons[event.code]
        direction= buttons_val[event.value]
        print(button, direction)

        print("")
    
    if event.type == ecodes.EV_ABS and event.code in absolutes:                     # leftpad, joystick motion, or L2/R2 triggers
            action, value = absolutes[event.code], event.value
            if event.code in [0, 1, 3, 4]:                                              # joystick motion
                if event.code in [0, 1]:                                                # left joystick moving
                    update_left_joystick_position(event)
                elif event.code in [3, 4]:                                              # right joystick moving
                    update_right_joystick_position(event)

                if event.value > (CENTER - BLIND) and event.value < (CENTER + BLIND):   # skip printing the jittery center for the joysticks
                    continue
                        
                print(f'{left_joystick}, {right_joystick}')
                continue

            if event.code in [2, 5]:                                                    # L2/R2 triggers
                print(f'{action} {value}')
            elif event.code in [16, 17]:                                                # leftpad (d-pad) action
                action = decode_leftpad(event)
                print(action)
                print("")
