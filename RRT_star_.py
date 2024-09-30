import pygame
import RRTbasepy
import time


def main():
    start = (100, 66)
    goal = (660, 500)
    dimensions = (600, 1000)
    obsdim = 50
    obsnum = 50

    pygame.init()
    map_ = RRTbasepy.RRTmap(start, goal, dimensions, obsdim, obsnum,'RRT* Path Planning')
    graph = RRTbasepy.RRTgraph(start, goal, dimensions, obsdim, obsnum)
    # tạo vật cản
    obstacles = graph.makeObs()
    # vẽ vật cản
    map_.drawmap(obstacles)
    iteration = 0
    t1 = time.time()
    while not graph.path_to_goal():
        eslaped = time.time() - t1
        t1 = time.time()
        if eslaped > 10:
            raise
        if iteration % 10 == 0:
            n = graph.number_of_nodes()
            x, y, parent = graph.bias(goal)
            pygame.draw.circle(map_.map, map_.red, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            # print(graph.number_of_nodes()-n)
            if graph.number_of_nodes()-n == 1:
                new_node = graph.number_of_nodes()-1
                neighbors = graph.near_neighbors(new_node, 90)
                graph.rewire(map_.map, new_node, neighbors)

            # time.sleep(0.05)
        else:
            n = graph.number_of_nodes()
            x, y, parent = graph.expand()
            pygame.draw.circle(map_.map, map_.grey, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            if graph.number_of_nodes() - n == 1:
                new_node = graph.number_of_nodes()-1
                neighbors = graph.near_neighbors(new_node, 90)
                graph.rewire(map_.map, new_node, neighbors)
                # pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                #                  (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            # time.sleep(0.05)
        # time.sleep(0.05)
        pygame.display.update()
        iteration += 1
    # map_.drawpath(graph.getPathcoords())
    for i in range(0, len(graph.getPathcoords()) - 1):
        pygame.draw.line(map_.map, (255, 255, 0), graph.getPathcoords()[i], graph.getPathcoords()[i + 1],
                         map_.edgeThickness + 5)
        time.sleep(0.03)
        pygame.display.update()
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == '__main__':
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False
