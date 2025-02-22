#!/usr/bin/python3
import sys 
import socket
import math
from tkinter import *
from pynput import keyboard

# ----------------------------------------------------------------------
# 1.0 Process the command line arguments, to get the port number.
if ( len(sys.argv) == 2) :
    trick_varserver_port = int(sys.argv[1])
else :
    print( "Usage: vsclient <port_number>")
    sys.exit()

# ----------------------------------------------------------------------
# 2.0 Set client Parameters.
HEIGHT, WIDTH = 800, 1000   # Canvas Dimensions
MARGIN = 20                # Margins around the axes
SCALE = 50                  # Scale = 50 pixels per meter
ballRadius = 15             # Ball radius in pixels

# ----------------------------------------------------------------------
# 3.0 Create constants for clarity.
MODE_FREEZE = 1 
MODE_RUN = 5 

# ----------------------------------------------------------------------
# 4.0 Create a variable to indicate that we want to "fire" the robot,
#     and a callback function to set it.
fireCommand = False
def botLaunch():
    global fireCommand
    fireCommand = True

# ----------------------------------------------------------------------
# 5.0 Create the GUI

# 5.1 Create a Canvas to draw on.
tk = Tk()
canvas = Canvas(tk, width=WIDTH, height=HEIGHT)
tk.title("Robot Display")
canvas.pack()

# 5.2  Add a FIRE button, whose callback sets the fireCommand variable.
buttonFrame = Frame()
buttonFrame.pack(side=BOTTOM)
fireButton = Button(buttonFrame,text="Start",command=botLaunch)
fireButton.pack(side=LEFT)


# 5.5 Create coordinate axes on the canvas.
xAxis = canvas.create_line(MARGIN,HEIGHT-MARGIN,WIDTH,HEIGHT-MARGIN)
yAxis = canvas.create_line(MARGIN,HEIGHT-MARGIN,MARGIN,0)

# 5.6 Create an oval object to represent the robotball.
#robotBall = canvas.create_rectangle(0,0,ballRadius,ballRadius, fill="orange")
robotBall = canvas.create_oval(0,0,ballRadius, ballRadius, fill="orange")

# 5.7 Create a text field on the canvas for the simulation mode display.
modeText = canvas.create_text(WIDTH/2, 20, text="--unknown-mode--")

# 5.8 Create text fields on the canvas for time and position of impact display.
impactTimeText = canvas.create_text(WIDTH/2, 40, text="")
impactPosText =  canvas.create_text(WIDTH/2, 60, text="")

# ----------------------------------------------------------------------
# 6.0 Connect to the variable server.
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect( ("localhost", trick_varserver_port) )
insock = client_socket.makefile("r")

# ----------------------------------------------------------------------
# 7.0 Request the robot ball position, the sim mode, and the impact info.
client_socket.send( b"trick.var_set_client_tag(\"myvsclient\") \n")
client_socket.send( b"trick.var_debug(3)\n" )
client_socket.send( b"trick.var_pause()\n" )
client_socket.send( b"trick.var_ascii()\n" )
client_socket.send( b"trick.var_add(\"dyn.ball.pos[0]\") \n" +
                    b"trick.var_add(\"dyn.ball.pos[1]\") \n" +
                    b"trick.var_add(\"trick_sys.sched.mode\")\n" +
                    b"trick.var_add(\"dyn.ball.tiltrate[0]\") \n" +
                    b"trick.var_add(\"dyn.ball.tiltrate[1]\") \n" +
                    b"trick.var_add(\"dyn.ball.acc[0]\") \n" +
                    b"trick.var_add(\"dyn.ball.acc[1]\") \n")
client_socket.send( b"trick.var_unpause()\n" )


def on_key(key):
    if key == keyboard.KeyCode.from_char('w'):
        client_socket.send( b"dyn.ball.motor[1] = 1 \n")
    elif key == keyboard.KeyCode.from_char('s'):
        client_socket.send( b"dyn.ball.motor[1] = -1 \n")
    elif key == keyboard.KeyCode.from_char('a'):
        client_socket.send( b"dyn.ball.motor[0] = -1 \n")
    elif key == keyboard.KeyCode.from_char('d'):
        client_socket.send( b"dyn.ball.motor[0] = 1 \n")

def off_key(key):
    if key == keyboard.KeyCode.from_char('w'):
        client_socket.send( b"dyn.ball.motor[1] = 0 \n")
        print("up")
    elif key == keyboard.KeyCode.from_char('s'):
        client_socket.send( b"dyn.ball.motor[1] = 0 \n")
    elif key == keyboard.KeyCode.from_char('a'):
        client_socket.send( b"dyn.ball.motor[0] = 0 \n")
    elif key == keyboard.KeyCode.from_char('d'):
        client_socket.send( b"dyn.ball.motor[0] = 0 \n")



board = keyboard.Listener(on_press=on_key, on_release=off_key)
board.start()

# ----------------------------------------------------------------------
# 8.0 Repeatedly read and process the responses from the variable server.
while(True):
    # 8.1 Read the response line from the variable server.
    line = insock.readline()
    if line == '':
        break

    # 8.2 Split the line into an array of value fields.
    field = line.split("\t")

    # 8.3 Get the position of the ball and update it on the canvas.
    x,y = float(field[1]), float(field[2])
    cx,cy = (x*SCALE+MARGIN), (HEIGHT-y*SCALE-MARGIN)
    canvas.coords(robotBall,cx-ballRadius,cy-ballRadius,cx+ballRadius,cy+ballRadius)

    # 8.4 Get and display current Sim Mode.
    simMode = int(field[3])
    if simMode == MODE_FREEZE:
        canvas.itemconfigure(modeText, fill="blue", text="FREEZE")
    elif simMode == MODE_RUN:
        canvas.itemconfigure(modeText, fill="red", text="RUN")
    else:
        canvas.itemconfigure(modeText, text="--unknown-mode--")

    # 8.5 When impact occurs display the impact info, and command the sim from RUN mode to FREEZE mode.

    # 8.6 When the "Fire" button is pressed, command the sim from FREEZE mode to RUN mode.
    if simMode == MODE_FREEZE:
        if fireCommand:
            fireCommand = False
            fireButton.config(state=DISABLED)
            # 8.6.1 Command the sim to assign the slider values to init_speed, and init_angle.
            # 8.6.2 Command the sim to re-run the robot_init job.
            client_socket.send( b"trick.robot_init( dyn.ball )\n")
            # 8.6.3 Command the sim to RUN mode.
            client_socket.send( b"trick.exec_run()\n")
    
    # 8.7 Update the Tk graphics.
    tk.update()

# ----------------------------------------------------------------------
# 9.0 Keep the window open, when the data stops.
tk.mainloop()