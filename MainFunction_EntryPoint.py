import cv2
import numpy as np
from time import sleep
import argparse

from env import Environment, Parking1
from HybridAimplementation import PathPlanning, ParkPathPlanning, interpolate_path
from control import Car_Dynamics, MPC_Controller


if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument('--x_start', type=int, default=4)
    parser.add_argument('--y_start', type=int, default=35)
    parser.add_argument('--psi_start', type=int, default=0)
    parser.add_argument('--x_end', type=int, default=90)
    parser.add_argument('--y_end', type=int, default=80)
   

    args = parser.parse_args()
    parking_spot = input(" Enter the parking spot out of the 24 position ")
    

   
    start = np.array([args.x_start, args.y_start])
    end   = np.array([args.x_end, args.y_end])
    
    parking1 = Parking1(int(parking_spot))
    end, obs = parking1.generate_obstacles()


    env = Environment(obs)
    my_car = Car_Dynamics(start[0], start[1], 0, np.deg2rad(args.psi_start), length=2, dt=0.2)
    MPC_HORIZON = 5
    controller = MPC_Controller()
   

    res = env.render(my_car.x, my_car.y, my_car.psi, 0)
    cv2.imshow('environment', res)
    key = cv2.waitKey(1)

    park_path_planner = ParkPathPlanning(obs)
    path_planner = PathPlanning(obs)

    print('planning park scenario ...')
    new_end, park_path, ensure_path1, ensure_path2 = park_path_planner.generate_park_scenario(int(my_car.x),int(my_car.y),int(end[0]),int(end[1]))

    print('routing to destination ...')
    path = path_planner.plan_path(int(my_car.x),int(my_car.y),int(new_end[0]),int(new_end[1]))
    path = np.vstack([path, ensure_path1])

    print('interpolating ...')
    interpolated_path = interpolate_path(path, sample_rate=5)
    interpolated_park_path = interpolate_path(park_path, sample_rate=2)
    interpolated_park_path = np.vstack([ensure_path1[::-1], interpolated_park_path, ensure_path2[::-1]])

    env.draw_path(interpolated_path)
    env.draw_path(interpolated_park_path)

    final_path = np.vstack([interpolated_path, interpolated_park_path, ensure_path2])


    print('driving to destination ...')
    MPC_reset = False
    try:
        for i,point in enumerate(final_path):

                acc, delta = controller.optimize(my_car, final_path[i:i+MPC_HORIZON])
                my_car.update_state(my_car.move(acc,  delta))
                res = env.render(my_car.x, my_car.y, my_car.psi, delta)
               
                cv2.imshow('environment', res)
                key = cv2.waitKey(1)
                if key == ord('s'):
                    cv2.imwrite('res.png', res*255)

                if my_car.y <= 0: 
                    print('generate obstacles')
                    end, obs = parking1.generate_obstacles(time=i)
                    env = Environment(obs)
                    park_path_planner = ParkPathPlanning(obs)
                    path_planner = PathPlanning(obs)

                    try:
                        print('planning park scenario ...')
                        new_end, park_path, ensure_path1, ensure_path2 = park_path_planner.generate_park_scenario(round(my_car.x),
                                                                                                                  round(my_car.y),
                                                                                                                  int(end[0]),
                                                                                                                  int(end[1]))
                        print('routing to destination ...')
                        path = path_planner.plan_path(round(my_car.x), round(my_car.y), int(new_end[0]), int(new_end[1]))
                        path = np.vstack([path, ensure_path1])

                        print('interpolating ...')
                        interpolated_path = interpolate_path(path, sample_rate=5)
                        interpolated_park_path = interpolate_path(park_path, sample_rate=2)
                        interpolated_park_path = np.vstack([ensure_path1[::-1], interpolated_park_path, ensure_path2[::-1]])

                        env.draw_path(interpolated_path)
                        env.draw_path(interpolated_park_path)

                        final_path = np.vstack([interpolated_path, interpolated_park_path, ensure_path2])
                    except Exception as e:
                        env.draw_path(interpolated_path)
                        env.draw_path(interpolated_park_path)
                        print(e)
                else: 
                    if not MPC_reset:
                        controller = MPC_Controller()
                    MPC_reset = True
                    env.draw_path(interpolated_path)
                    env.draw_path(interpolated_park_path)
                   
    except Exception as e2:
       
        res = env.render(my_car.x, my_car.y, my_car.psi, 0)
       
        cv2.imshow('environment', res)
        key = cv2.waitKey()
        

        cv2.destroyAllWindows()

