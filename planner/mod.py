import numpy

class Module:
    grid = []

    def which_car(self, cars, route_todo, routes):
        raise NotImplementedError()

    def new_job(self, cars, routes):
        raise NotImplementedError()

    ### interface for blocking and freeing ways

    def get_Map(self):
        return self.grid

    def set_Map(self,new_Map):
        self.grid = new_Map

    def block_Map(self,pos_x, pos_y):
        self.grid[pos_x, pos_y] = -1

    def free_Map(self):
        self.grid.fill(0)

    def print_Map(self):
        print (self.grid)