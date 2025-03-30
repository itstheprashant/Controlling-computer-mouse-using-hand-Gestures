import cv2
import mediapipe as mp
import pyautogui
import math
import numpy as np
import time
from pynput.keyboard import Key, Controller as KeyboardController
from pynput.mouse import Button, Controller as MouseController
keyboard = KeyboardController()
mouse = MouseController()
# Disable PyAutoGUI fail-safe
pyautogui.FAILSAFE = False

# Initialize MediaPipe Hand Module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Constants for screen dimensions (adjust as needed)
SCREEN_WIDTH = 1920  # Screen width in pixels
SCREEN_HEIGHT = 1080  # Screen height in pixels
prev_fingertip_coords = None
# Constants for cursor movement smoothing
# SMOOTHING_FACTOR = 0.2  Adjust the smoothing factor (0.0 to 1.0) as needed
def zoom_in():
    # Simulate pressing the 'Ctrl' key and the '+' key to zoom in 
    pyautogui.hotkey('ctrl', '+')
    time.sleep(0.5)

def zoom_out():
    # Simulate pressing the 'Ctrl' key and the '-' key to zoom out 
    pyautogui.hotkey('ctrl', '-')
    time.sleep(0.5)
#############################################################################################################
def detect_touching(landmarks):
    finger_landmarks = landmarks.landmark

    # Define landmarks for thumb, index, middle, ring, and pinky fingers
    thumb_tip_id = 4
    index_tip_id = 8
    middle_tip_id = 12
    ring_tip_id = 16
    pinky_tip_id = 20

    if len(finger_landmarks) >= 21:  # Check if enough landmarks are detected
        thumb_tip = np.array([finger_landmarks[thumb_tip_id].x, finger_landmarks[thumb_tip_id].y])
        index_tip = np.array([finger_landmarks[index_tip_id].x, finger_landmarks[index_tip_id].y])
        middle_tip = np.array([finger_landmarks[middle_tip_id].x, finger_landmarks[middle_tip_id].y])
        ring_tip = np.array([finger_landmarks[ring_tip_id].x, finger_landmarks[ring_tip_id].y])
        pinky_tip = np.array([finger_landmarks[pinky_tip_id].x, finger_landmarks[pinky_tip_id].y])

        # Define a distance threshold for considering fingers as touching
        distance_threshold = 0.05  # Adjust this threshold as needed

        # Calculate the Euclidean distance between thumb and fingers
        index_distance = np.linalg.norm(thumb_tip - index_tip)
        middle_distance = np.linalg.norm(thumb_tip - middle_tip)
        ring_distance = np.linalg.norm(thumb_tip - ring_tip)
        pinky_distance = np.linalg.norm(thumb_tip - pinky_tip)

        # Check if any finger is touching
        if index_distance < distance_threshold:
            return 'index'
        elif middle_distance < distance_threshold:
            return 'middle'
        elif ring_distance < distance_threshold:
            return 'ring'
        elif pinky_distance < distance_threshold:
            return 'pinky'

    return None
###########################################################################################################

def detect_open_fingers(landmarks, frame):
    finger_landmarks = landmarks.landmark
    open_fingers = []

    # Define landmarks for thumb, index, middle, ring, and pinky fingers
    finger_tip_ids = [4, 8, 12, 16, 20]

    if len(finger_landmarks) >= 21:  # Check if enough landmarks are detected
        for finger_tip_id in finger_tip_ids:
            tip = finger_landmarks[finger_tip_id]
            dip = finger_landmarks[finger_tip_id - 1]
            pip = finger_landmarks[finger_tip_id - 2]

            # Adjust for flipped frame: Check if fingertip (tip) is higher (in y) than PIP and also to the right (in x)
            if tip.x < pip.x and tip.x < dip.x and tip.y < pip.y and tip.y < dip.y:
                if finger_tip_id == 4:
                    open_fingers.append('Thumb')
                else:
                    open_fingers.append(finger_name(finger_tip_id))

    return open_fingers

def finger_name(finger_tip_id):
    finger_names = ['Index', 'Middle', 'Ring', 'Pinky']
    # Calculate the correct index
    return finger_names[(finger_tip_id - 8) // 4]
# Variables to track cursor position and velocity
cursor_position = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
cursor_velocity = [0, 0]

# Custom colors for fingertips
finger_tip_colors = [(0, 0, 255),  # Red for thumb
                     (0, 255, 0),  # Green for index finger
                     (255, 0, 0),  # Blue for middle finger
                     (255, 255, 0),  # Yellow for ring finger
                     (0, 255, 255)  # Cyan for pinky finger
                    ]

# Function to get fingertip coordinates for a hand
def get_fingertip_coordinates(landmarks):
    fingertip_coordinates = []
    for idx, landmark in enumerate(landmarks.landmark):
        if idx ==8:  # Indexes for the tips of the five fingers
            h, w, c = frame.shape
            cx, cy = int(landmark.x * w), int(landmark.y * h)
            fingertip_coordinates.append((cx, cy))
    return fingertip_coordinates

 
# Function to control the cursor based on finger movement
movement_distance_cm = 9.0  # Adjust this value as needed
# ... (previous code)

# Variables for low-pass filtering
filter_alpha = 1.0  # Smoothing factor (0.0 to 1.0)
# ... (previous code)

# Function to get distance between two points
def calculate_distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

# ... (previous code)

# Function to check if the middle finger is folded
def calculate_angle(l1, l2, l3):
    x1, y1 = l1.x, l1.y
    x2, y2 = l2.x, l2.y
    x3, y3 = l3.x, l3.y

    # Calculate the vectors between the landmarks
    vec1 = (x1 - x2, y1 - y2)
    vec2 = (x3 - x2, y3 - y2)

    # Calculate the dot product and the magnitudes of the vectors
    dot_product = vec1[0] * vec2[0] + vec1[1] * vec2[1]
    magnitude1 = (vec1[0] ** 2 + vec1[1] ** 2) ** 0.5
    magnitude2 = (vec2[0] ** 2 + vec2[1] ** 2) ** 0.5

    # Calculate the angle in degrees
    angle_rad = math.acos(dot_product / (magnitude1 * magnitude2))
    angle_deg = math.degrees(angle_rad)

    return angle_deg
def is_middle_finger_folded(hand_landmarks):
    # Get the landmarks of the middle finger
    middle_finger_landmarks = [
        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP],
        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP],
        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    ]

    # Calculate the angle between the finger joints
    angle1 = calculate_angle(middle_finger_landmarks[0], middle_finger_landmarks[1], middle_finger_landmarks[2])

    # Define a threshold angle (adjust as needed)
    threshold_angle = 90.0  # Adjust this value to control the sensitivity

    # Check if the middle finger is folded (angle is less than the threshold)
    return angle1 < threshold_angle

    # Check if the middle finger is folded
    return all(angle < 0 for angle in angles)

# Function to control cursor movement and click
def control_cursor_and_click(fingertip_coords, hand_landmarks):
    global cursor_position, cursor_velocity, prev_fingertip_coords

    if len(fingertip_coords) >= 1:
        # Check if the middle finger is folded
        if 2==1:
            print("check")
        else:
            # Move the cursor with the index finger
            if prev_fingertip_coords is not None:
                dx = fingertip_coords[0][0] - prev_fingertip_coords[0][0]
                dy = fingertip_coords[0][1] - prev_fingertip_coords[0][1]

                movement_x = dx * movement_distance_cm * SCREEN_WIDTH / 1920
                movement_y = dy * movement_distance_cm * SCREEN_HEIGHT / 1080

                cursor_velocity = (
                    filter_alpha * movement_x + (1 - filter_alpha) * cursor_velocity[0],
                    filter_alpha * movement_y + (1 - filter_alpha) * cursor_velocity[1],
                )

                cursor_position = (
                    cursor_position[0] + int(cursor_velocity[0]),
                    cursor_position[1] + int(cursor_velocity[1]),
                )

                cursor_position = (
                    min(max(cursor_position[0], 0), SCREEN_WIDTH),
                    min(max(cursor_position[1], 0), SCREEN_HEIGHT),
                )

                pyautogui.moveTo(cursor_position[0], cursor_position[1])

        prev_fingertip_coords = fingertip_coords
    else:
        prev_fingertip_coords = None

# Capture Video Stream (or Load Image)
cap = cv2.VideoCapture(0)  # 0 for the default webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
while True:
    ret, frame = cap.read()  # Read a frame from the webcam
    frame=cv2.flip(frame,1)
    # Convert the BGR frame to RGB (MediaPipe uses RGB format)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame to detect hands
    results = hands.process(rgb_frame)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Get fingertip coordinates for the hand
            fingertip_coords = get_fingertip_coordinates(hand_landmarks)

            # Draw circles for fingertips with custom colors
            for i, coord in enumerate(fingertip_coords):
                color = finger_tip_colors[i]
                cv2.circle(frame, coord, radius=10, color=color, thickness=-1)  # Draw a filled circle
                touching_finger = detect_touching(hand_landmarks)
                open_fingers = detect_open_fingers(hand_landmarks, frame)

                if touching_finger == 'index':
                    mouse.scroll(0,0.75)
                    
                elif touching_finger == 'middle':
                    mouse.scroll(0,-0.75)
                elif touching_finger == 'ring':
                    zoom_in()
                    
                elif touching_finger == 'pinky':
                    zoom_out() 
                                 
                
                elif detect_open_fingers(hand_landmarks, frame)==['Thumb', 'Index', 'Pinky']:
                    keyboard.press(Key.media_volume_up)
                    keyboard.release(Key.media_volume_up)
                elif detect_open_fingers(hand_landmarks, frame)==['Thumb', 'Pinky']:
                    keyboard.press(Key.media_volume_down)
                    keyboard.release(Key.media_volume_down) 
                # elif detect_open_fingers(hand_landmarks, frame)==['Thumb', 'Index','Ring' 'Pinky']:
                #     Button.right()

                elif is_middle_finger_folded(hand_landmarks):
                    pyautogui.click()  # Execute a click action
                # Control the cursor based on finger movement
                control_cursor_and_click(fingertip_coords, hand_landmarks)
                # cv2.putText(frame, f'Open Fingers: {", ".join(open_fingers)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # if detect_open_fingers(hand_landmarks, frame)==['Thumb', 'Index', 'Pinky']:
                #     keyboard.press(Key.media_volume_up)
                #     keyboard.release(Key.media_volume_up)
                # elif detect_open_fingers(hand_landmarks, frame)==['Thumb', 'Pinky']:
                #     keyboard.press(Key.media_volume_down)
                #     keyboard.release(Key.media_volume_down)                

            # if is_middle_finger_folded(hand_landmarks):
            #         cv2.putText(frame, "Middle Finger Folded", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Draw hand landmarks
            mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Display the frame with landmarks and smoothed cursor movement
    cv2.imshow("Hand Tracking", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press Esc to exit
        break

cap.release()
cv2.destroyAllWindows()





