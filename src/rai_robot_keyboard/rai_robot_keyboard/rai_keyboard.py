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
Control Your carrrrrrrrrr!
---------------------------
Moving around:
   q    w    e
   a    x    d
   z    s    c
"""
e = """
Communications Failed
"""

moveBindings = {
        'w':( 1, 0),
        'e':( 1,-1),
        'a':( 0, 1),
        'd':( 0,-1),
        'q':( 1, 1),
        's':(-1, 0),
        'c':(-1,1),
        'z':(-1,-1),
}

speedBindings={
        'u':(1.1,1),
        'i':(0.9,1),
}

speed = 0.2 # m/s
turn  = 1   # rad/
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
    th     = 0.0
    count  = 0.0
    target_speed = 0.0
    target_turn  = 0.0
    target_HorizonMove = 0.0
    control_speed = 0.0
    control_turn  = 0.0
    control_HorizonMove = 0.0
    Omni = 0
    try:
        print(msg)
        print(print_vels(speed, turn))
        while(1):
            key = get_key(settings)
            if key=='b':         
                Omni=~Omni
                if Omni: 
                    print("Switch to OmniMode")
                    moveBindings['.']=[-1,-1]
                    moveBindings['m']=[-1, 1]
                else:
                    print("Switch to CommonMode")
                    moveBindings['.']=[-1, 1]
                    moveBindings['m']=[-1,-1]
            
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn  = turn  * speedBindings[key][1]
                count = 0
                print(print_vels(speed,turn))

            elif key == ' ' or key == 'x' :
                x  = 0
                th = 0.0
                control_speed = 0.0
                control_turn  = 0.0
                HorizonMove   = 0.0

            else:
                count = count + 1
                if count > 4:
                    x  = 0
                    th = 0.0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn  = turn * th
            target_HorizonMove = speed*th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.5 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.5 )
            else:
                control_turn = target_turn

            if target_HorizonMove > control_HorizonMove:
                control_HorizonMove = min( target_HorizonMove, control_HorizonMove + 0.1 )
            elif target_HorizonMove < control_HorizonMove:
                control_HorizonMove = max( target_HorizonMove, control_HorizonMove - 0.1 )
            else:
                control_HorizonMove = target_HorizonMove
         
            twist = Twist()
            if Omni==0:
                twist.linear.x  = control_speed; twist.linear.y = 0.0;  twist.linear.z = 0.0
                twist.angular.x = 0.0;             twist.angular.y = 0.0; twist.angular.z = control_turn
            else:
                twist.linear.x  = control_speed; twist.linear.y = control_HorizonMove; twist.linear.z = 0.0
                twist.angular.x = 0.0;             twist.angular.y = 0.0;                  twist.angular.z = 0.0
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
