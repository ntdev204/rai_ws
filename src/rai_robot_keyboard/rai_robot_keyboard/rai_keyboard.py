import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


msg = """
Control Your Rai Robot!
---------------------------
Moving around:
   q    w    e
   a    x    d
   z    s    c

w/s : linear x (forward/backward)
a/d : linear y (left/right strafe)
q/e : diagonal forward left/right
z/c : diagonal backward left/right
r/t : rotate in place (left/right)
x   : force stop

u/i : increase/decrease max speeds

CTRL-C to quit
"""

e = """
Communications Failed
"""

# Binding: (x, y, th)
moveBindings = {
        'w':( 1, 0, 0),
        's':(-1, 0, 0),
        'a':( 0, 1, 0),
        'd':( 0,-1, 0),
        'q':( 1, 1, 0),
        'e':( 1,-1, 0),
        'z':(-1, 1, 0), 
        'c':(-1,-1, 0),
        'r':( 0, 0, 1),
        't':( 0, 0,-1),
}

speedBindings={
        'u':(1.1,1.1),
        'i':(0.9,0.9),
}

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(speed, turn):
    print('currently:\tspeed {0}\t turn {1} '.format(
        speed,
        turn))

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('rai_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    speed = 0.2
    turn  = 1.0
    x      = 0.0
    y      = 0.0
    th     = 0.0
    count  = 0.0
    target_speed = 0.0
    target_lat_speed = 0.0
    target_turn  = 0.0
    control_speed = 0.0
    control_lat_speed = 0.0
    control_turn  = 0.0

    try:
        print(msg)
        print(print_vels(speed, turn))
        while(1):
            key = get_key(settings)
            
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                y  = moveBindings[key][1]
                th = moveBindings[key][2]
                count = 0

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn  = turn  * speedBindings[key][1]
                count = 0
                print(print_vels(speed,turn))

            elif key == ' ' or key == 'x' :
                x  = 0.0
                y  = 0.0
                th = 0.0
                control_speed = 0.0
                control_lat_speed = 0.0
                control_turn  = 0.0

            else:
                count = count + 1
                if count > 4:
                    x  = 0.0
                    y  = 0.0
                    th = 0.0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_lat_speed = speed * y
            target_turn  = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                control_speed = target_speed
                
            if target_lat_speed > control_lat_speed:
                control_lat_speed = min( target_lat_speed, control_lat_speed + 0.1 )
            elif target_lat_speed < control_lat_speed:
                control_lat_speed = max( target_lat_speed, control_lat_speed - 0.1 )
            else:
                control_lat_speed = target_lat_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.5 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.5 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed
            twist.linear.y = control_lat_speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_turn
            
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
