#!/usr/bin/env python
import logging
from time import sleep

import libardrone.libardrone as libardrone

import cv2


def main():
    drone = libardrone.ARDrone2()
    drone.reset()
    drone.set_speed(0.1)

    W, H = 360, 640

    try:
        running = True

        last_pressed_keys = []

        while running:
            pressed_key = cv2.waitKey(15) & 0xFF

            last_pressed_keys.append(pressed_key)

            if len(last_pressed_keys) > 15:
                last_pressed_keys.pop(0)

            key_up = all(last_pressed_key == 255 for last_pressed_key in last_pressed_keys)

            print last_pressed_keys, key_up

            very_last_pressed_key = None

            if not key_up:
                for last_pressed_key in last_pressed_keys:
                    if last_pressed_key != 255:
                        very_last_pressed_key = last_pressed_key

                print very_last_pressed_key

                if very_last_pressed_key == ord('q'):  # Q
                    running = False
                elif very_last_pressed_key == ord('w'):  # W
                    drone.move_forward()
                elif very_last_pressed_key == ord('s'):  # S
                    drone.move_backward()
                elif very_last_pressed_key == ord('a'):  # A
                    drone.move_left()
                elif very_last_pressed_key == ord('d'):  # D
                    drone.move_right()

                elif very_last_pressed_key == 0:  # Key Up
                    drone.move_up()
                elif very_last_pressed_key == 1:  # Key Down
                    drone.move_down()
                elif very_last_pressed_key == 2:  # Key Left
                    drone.turn_left()
                elif very_last_pressed_key == 3:  # Key Right
                    drone.turn_right()
                elif very_last_pressed_key == 32:  # Space
                    drone.land()
                elif very_last_pressed_key == 13:  # Enter
                    drone.takeoff()

                elif very_last_pressed_key == ord('i'):
                    HOST = 'localhost'
                    PORT = '5000'

                    # requests.post(url='%s:%s/models/images/classification/classify_one.json -XPOST -F job_id=20160809-164902-44b3 -F image_file=@/home/lukas/5.jpg' % (
                    #     HOST,
                    #     PORT
                    # ),data=)

                else:
                    print very_last_pressed_key
            else:
                drone.hover()

            image = drone.get_image()

            # print image

            image = cv2.cvtColor(image, cv2.IMREAD_COLOR)  # COLOR_BGR2RGB

            try:
                nav_data = drone.get_navdata()

                nav_data = nav_data[0]

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_size = 0.5

                cv2.putText(image, 'State: %s' % nav_data['ctrl_state'], (5, 15), font, font_size, (255, 255, 255))

                cv2.putText(image, 'Phi (+Roll Right/- Roll Left): %.0f' % nav_data['phi'], (5, 30), font, font_size, (255, 255, 255))
                cv2.putText(image, 'Psi (+Turn Right/- Turn Left): %.0f' % nav_data['psi'], (5, 45), font, font_size, (255, 255, 255))
                cv2.putText(image, 'Theta (+Up/-Down): %.0f' % nav_data['theta'], (5, 60), font, font_size, (255, 255, 255))

                cv2.putText(image, 'Altitude: %.0f' % nav_data['altitude'], (5, 75), font, font_size, (255, 255, 255))
                cv2.putText(image, 'vx/vy/vz: %.0f/%.0f/%.0f' % (nav_data['vx'], nav_data['vy'], nav_data['vz']), (5, 90), font, font_size, (255, 255, 255))

                cv2.putText(image, 'Battery: %.0f%%' % nav_data['battery'], (5, 105), font, font_size, (255, 255, 255))

                # if nav_data['battery'] == 0:
                #     print 'No battery left or not connected with correct WiFi!!'
                #     return

                cv2.putText(image, '# Frames (?): %.0f' % nav_data['num_frames'], (5, 120), font, font_size, (255, 255, 255))
                cv2.putText(image, 'Command: %s' % (very_last_pressed_key if very_last_pressed_key else '-',), (5, 135), font, font_size, (255, 255, 255))

            except Exception, e:
                print 'Exception happened while trying to access nav data:'
                logging.error(e, exc_info=True)

            cv2.rectangle(image, (H / 2 - 25, W / 2 - 25), (H / 2 + 25, W / 2 + 25), (0, 255, 0), 2, )

            # cv2.blur(image, (100, 100), (-1, -1))
            # mage = cv2.bilateralFilter(image, 9, 75, 75)

            cv2.imshow('Live Drone Cam (Jeronimo)', image)

            # sleep(0.01)
            # drone.hover()

    except Exception, e:
        print 'Exception happened in main while loop:'
        logging.error(e, exc_info=True)

    cv2.destroyAllWindows()

    print("Shutting down...")
    drone.reset()
    drone.halt()
    print("Shut down.")


def get_bouding_rect():
    return (100, 100)


if __name__ == '__main__':
    main()
