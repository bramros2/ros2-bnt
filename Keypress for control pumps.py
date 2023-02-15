
import rospy
from std_msgs.msg import String

def on_key_pressed(key):
    # start motor corresponding to the pressed key
    if key == 'x':
        start_motor('x')
    elif key == 'y':
        start_motor('y')
    elif key == 'z':
        start_motor('z')

def on_key_released(key):
    # stop motor corresponding to the released key
    if key == 'x':
        stop_motor('x')
    elif key == 'y':
        stop_motor('y')
    elif key == 'z':
        stop_motor('z')

# define keyboard input callback function
def keyboard_input_callback(data):
    # get the pressed key
    key = data.data

    # check if the key is one of the motor control keys
    if key in ['x', 'y', 'z']:
        on_key_pressed(key)
    else:
        # ignore other keys
        pass

# main function
def main():
    # create a node
    rospy.init_node('motor_control_node')

    # subscribe to keyboard input topic
    rospy.Subscriber('/keyboard_input', String, keyboard_input_callback)

    # start the motor control loop
    rospy.spin()

if __name__ == '__main__':
    main()