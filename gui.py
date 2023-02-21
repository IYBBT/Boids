
from boids import Boid
import numpy as np, pygame
from numpy.linalg import norm

pygame.init()
backgroud_color = (135,206,235)
boid_color = (48,48,48)

class Boid_Model:
    def __init__(self, boid):
        self.p1 = np.array([[0],
                           [0]]).reshape(2,1)
        
        self.p2 = np.array([[5],
                           [-15]]).reshape(2,1)
        
        self.p3 = np.array([[-5],
                           [-15]]).reshape(2,1)
        
        if norm(boid.Vel) > 0:
            ux = boid.Vel[0, 0] / norm(boid.Vel)
            angle = np.arcsin([ux])
            m = [[np.cos(angle), np.sin(angle)],
                [-np.sin(angle), np.cos(angle)]]
            rmatrix = np.array(m).reshape(2,2)

            self.p1 = np.dot(rmatrix, self.p1)
            self.p2 = np.dot(rmatrix, self.p2)
            self.p3 = np.dot(rmatrix, self.p3)

        self.p1 += boid.Pos
        self.p2 += boid.Pos
        self.p3 += boid.Pos

    def points(self):
        p1 = (self.p1[0,0], self.p1[1,0])
        p2 = (self.p2[0,0], self.p2[1,0])
        p3 = (self.p3[0,0], self.p3[1,0])

        return (p1, p2, p3)

class Boids_GUI:
    def __init__(self, num_boids, wind_x, wind_y):
        self.boids = []
        for i in range(num_boids):
            self.boids.append(Boid(wind_x, wind_y))

        pygame.display.set_caption('Boids')
        self.screen = pygame.display.set_mode((wind_x, wind_y))

    def run(self):
        running = True
        while running:
            self.screen.fill(backgroud_color)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            for i in range(len(self.boids)):
                self.boids[i].update(self.boids)
                pygame.draw.polygon(self.screen, boid_color, Boid_Model(self.boids[i]).points())

            pygame.display.update()
        pygame.quit()

if __name__ == '__main__':
    GUI = Boids_GUI(50, 500, 500)
    GUI.run()
