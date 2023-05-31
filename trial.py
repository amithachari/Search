def find_closest_goal(neighbor, new_waypoints):
    # print(new_waypoints)
    closest = new_waypoints[0]
    # print(closest)
    length0 = abs(neighbor[0] - closest[0]) + abs(neighbor[1] - closest[1])
    for waypoint in new_waypoints:
        length = abs(neighbor[0] - waypoint[0]) + abs(neighbor[1] - waypoint[1])
        if length < length0:
            length0 = length
            closest = waypoint
    return length0

leng = find_closest_goal((2,2),[(0,0),(10,0),(0,10),(10,10)])
print(leng)