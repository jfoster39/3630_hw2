from pygame import*
quit
import random

class labyrinthe(list):
    ''
    def __init__(self,size):
        self.size = size
        labx,laby = size
        lx,ly = labx+2,laby+2
        ref = [1,lx,-1,-lx]
        l=[[random.randrange(lx+1,lx*(ly-1),lx)+random.randint(0,labx),random.choice(ref)]]
        L = list((0,0,0,0)*lx+((0,0,0,0)+(1,1,1,1)*labx+(0,0,0,0))*laby+(0,0,0,0)*lx)
        L = [L[i:i+4] for i in range(0,lx*ly*4,4)]
        self.extend(L)
        while l:
            for i in l:
                a = sum(i)
                b  = (1 if abs(i[1])==lx else lx)*random.choice((1,-1))
                if all(self[a]):
                    c = ref.index(i[1])
                    self[i[0]][c] = 0
                    i[0] = a
                    self[i[0]][c-2] = 0
                    if not random.randint(0,1): l.append([i[0],b])
                    if not random.randint(0,3): l.append([i[0],-b])
                else :
                    if all(self[i[0]+b]): l.append([i[0],b])
                    if all(self[i[0]-b]): l.append([i[0],-b])
                    l.remove(i)
        del(self[:lx])
        del(self[-lx:])
        del(self[::lx])
        del(self[lx-2::lx-1])

    def get_path(self,start,exit):
        pos = start
        d = 1
        path = [pos]
        ref = [1,self.size[0],-1,-self.size[0]]
        while pos != exit:
            for x in range( 0, 4 ):
                if( self[pos][x] == 0 ):
                    #calculate cost of going this direction
                    cost = self.current_cost( self.index_xy_conversion( pos ), x );
                    #add cost and location to queue



            #original code
            if self[pos][ref.index(d)-1] == 0:
                d = ref[ref.index(d)-1]
            if self[pos][ref.index(d)] == 0:
                pos = pos+d
                path.append(pos)
                i = path.index(pos)
                if i != len(path)-1:
                    del(path[i:-1])
            else: d = ref[ref.index(d)-3]
        return path

    def index_xy_conversion( self, index ):
        size = self.size[0];
        y = index / size;
        x = index % size;
        return [ x, y ];

    def xy_index_conversion( self, coords ):
        size = self.size[0];
        y = coords[1] * size;
        x = coords[0];
        return y + x;

    def distance_between_coords( self, coord1, coord2 ):
        import math;
        x1 = coord1[0];
        y1 = coord1[1];
        x2 = coord2[0];
        y2 = coord2[1];

        d  = math.sqrt( (x2 - x1)**2 + (y2-y1)**2 );
        return d;
 
    def current_cost( self, start, direction ):
        # start is the coordinate that you are starting at
        # direction is the direction that you are going
        # use 0-3 for the direction
        # go that direction as long as the only "open" choices - the one going in "direction" and the direction you came from
        # keep count of the number of spaces you can move
        # return array of form: [ x, y, distance_travelled ]
        # where x and y are the ending coord of the movement
        distance = 0;
        req = [1,1,1,1];
        x = start[0];
        y = start[1];
        keep_moving = 1;

        while( keep_moving ):
            if direction == 0: #move right
                if( x > 49 ):
                    break;
                req = [0,1,0,1];
                nextx = x+1;
                nexty = y;

            elif direction == 1: #move down
                if( y > 49 ):
                    break;
                req = [1,0,1,0];
                nextx = x;
                nexty = y+1;

            elif direction == 2: #move left
                if( x < 0 ):
                    break;
                req = [0,1,0,1];
                nextx = x-1;
                nexty = y;

            elif direction == 3: #move up
                if( y < 50 ):
                    break;
                req = [1,0,1,0];
                nextx = x;
                nexty = y-1;

            next_set = self[ self.xy_index_conversion( [nextx,nexty] )];

            if( next_set != req ):
                keep_moving = 0;
            else:
                distance = distance + 1;
                x = nextx;
                y = nexty;

        values = [distance, x, y];
        return values;

    def total_cost( self, start, goal, direction ):
        # start is current coord
        # goal is goal coord
        # direction is the binary of the direction of travel

        vals = self.current_cost( start, direction );
 
        destination = [ vals[0], vals[1] ];
        cost = vals[2];

        distance = self.distance_between_coords( destination, goal );

        total_cost = cost + distance;
        return total_cost;

    def get_image_and_rects(self,cellulesize,wallcolor=(0,0,0),celcolor=(255,255,255)):
        x,y = cellulesize
        image = Surface((x*(self.size[0]),y*self.size[1]))
        image.fill(wallcolor)
        rects = []
        for e,i in enumerate(self):
            rects.append(image.fill(celcolor,(e%(self.size[0])*cellulesize[0]+1-(not i[2]),e/(self.size[0])*cellulesize[1]+1-(not i[3]),cellulesize[0]-2+(not i[2])+(not i[0]),cellulesize[1]-2+(not i[1])+(not i[3]))))
        return image,rects

#****************************************************************************************
#****************************************************************************************
if __name__ == '__main__':
    me = Surface((5,5))
    me.fill(0xff0000)
    L = labyrinthe((50,50))
    labx,laby = 50,50
    screen = display.set_mode((L.size[0]*10,L.size[1]*10))
    image,rectslist = L.get_image_and_rects((10,10),wallcolor=0,celcolor=0xffffff)
    screen.blit(image,(0,0))
    start = random.randrange(len(L))
    exit = random.randrange(len(L))
    screen.fill(0x00ff00,rectslist[exit])
    screen.blit(me,rectslist[start])
    display.flip()
    while event.wait().type != QUIT:
        screen.fill(-1,rectslist[start])
        if key.get_pressed()[K_RIGHT] and not L[start][0]:
            start += 1
        if key.get_pressed()[K_LEFT] and not L[start][2]:
            start += -1
        if key.get_pressed()[K_UP] and not L[start][3]:
            start += -L.size[1]
        if key.get_pressed()[K_DOWN] and not L[start][1]:
            start += L.size[0]
        screen.fill(0xff0000,rectslist[start])
        display.flip()
        if start == exit : print 'YOU WIN'; break
        if key.get_pressed()[K_ESCAPE]:
            for i in L.get_path(start,exit)[1:-1]:
                screen.fill(0x0000ff,rectslist[i])
                display.update(rectslist[i])
                time.wait(20)
