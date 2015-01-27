from pygame import*
quit
import random
from Queue import *

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
        all_path = [] #all nodes in paths
        prev_path = [] #lists nodes in paths and their previous nodes
        total_cost = 0; #total cost is the total cost of getting to pos
        current_cost = 0; #current cost is the cost of pos + current action
        visited = set()
        frontier = PriorityQueue() #queue used for storing successors of current location
        #queue items: [heuristic_cost + cost to node, current node, cost to node, and prev node]
        frontier.put( ( 0, pos, 0, -1 ) ) # add first entry to PQ

        while frontier.empty != False :
            pos_node = frontier.get()
            pos = pos_node[1]
            total_cost = pos_node[2]
            prev_node = pos_node[3]

            if pos == exit :
                #path found
                #add exit to path
                all_path.append( pos );
                prev_path.append( [pos, prev_node] );

                #reconstruct shortest path
                path = [pos];
                current = prev_node;

                while( current != start ):
                    #find index of current
                    i = all_path.index(current);
                    #find previous node
                    prev = prev_path[i][1];

                    #add current to path
                    path.append( current );
                    current = prev;
                filled_path = self.fill_in_path( path );
                return filled_path;

            elif pos in visited :
                #skip already visited
                continue

            for x in range( 0, 4 ):
                #check all 4 directions
                if( self[pos][x] == 0 ):
                    pos_coord  = self.index_xy_conversion( pos  );
                    exit_coord = self.index_xy_conversion( exit );

                    #calculate cost of going this direction
                    vals = self.action_cost( pos_coord, x, exit );
                    destination = [ vals[0], vals[1] ];
                    action_cost = vals[2];

                    #calculate distance
                    distance = self.distance_between_coords( destination, exit_coord );

                    #add action cost to previous total cost
                    current_cost = total_cost + action_cost;

                    #add current cost and distance
                    heuristic_cost = current_cost + distance;

                    destination_index = self.xy_index_conversion( destination );

                    #add heuristic_cost, destination, current_cost, and prev node to queue
                    #queue is ordered by lowest heuristic_cos
                    frontier.put( ( heuristic_cost, destination_index, current_cost, pos ) )

            #add pos to visited to prevent duplicates
            visited.add( pos )

            # append to paths for path reconstruction
            all_path.append( pos );
            prev_path.append( [pos, prev_node] );

    def fill_in_path( self, path ):
        for i in range(0,len(path)-1 ):
            #draw from one node to next for path
            a = self.index_xy_conversion( path[i] );
            b = self.index_xy_conversion( path[i+1] );

            #fill in between a and b
            if( a[0] < b[0] ): #drawing right
                diff = b[0] - a[0] ;
                for j in range(1, diff+1):
                    path.append( self.xy_index_conversion( [a[0]+j, a[1]] ) );

            elif( a[0] > b[0] ): #drawing left
                diff = a[0] - b[0];
                for j in range(1, diff+1):
                    path.append( self.xy_index_conversion( [a[0]-j, a[1]] ) );

            elif( a[1] < b[1] ): #drawing down
                diff = b[1] - a[1] ;
                for j in range(1, diff+1):
                    path.append( self.xy_index_conversion( [a[0], a[1]+j] ) );

            elif( a[1] > b[1] ): #drawing up
                diff = a[1] - b[1];
                for j in range(1, diff+1):
                    path.append( self.xy_index_conversion( [a[0], a[1]-j] ) );
        return path;

    def action_cost( self, start, direction, goal ):
        # start is the coordinate that you are starting at
        # direction is the direction that you are going
        # use 0-3 for the direction
        # go that direction as long as the only "open" choices are the ones
        # going in the specified "direction" and the direction you came from
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
                nextx = x+1;
                nexty = y;
                if( x > 49 ):
                    break;
                req = [0,1,0,1];

                next_pos = self.xy_index_conversion( [nextx,nexty] );
                if( next_pos == goal ):
                    x = nextx;
                    y = nexty;
                    distance = distance + 1;
                    break;

                next_set = self[ next_pos ];
                if( next_set == [1,1,0,1] ):
                    return [ nextx, nexty, float( "inf" ) ];
            elif direction == 1: #move down
                nextx = x;
                nexty = y+1;
                if( y > 49 ):
                    break;
                req = [1,0,1,0];

                next_pos = self.xy_index_conversion( [nextx,nexty] );
                if( next_pos == goal ):
                    x = nextx;
                    y = nexty;
                    distance = distance + 1;
                    break;

                next_set = self[ next_pos ];
                if( next_set == [1,1,1,0] ):
                    return [ nextx, nexty, float( "inf" ) ];
            elif direction == 2: #move left
                nextx = x-1;
                nexty = y;
                if( x < 0 ):
                    break;
                req = [0,1,0,1];

                next_pos = self.xy_index_conversion( [nextx,nexty] );
                if( next_pos == goal ):
                    x = nextx;
                    y = nexty;
                    distance = distance + 1;
                    break;

                next_set = self[ next_pos ];
                if( next_set == [0,1,1,1] ):
                    return [ nextx, nexty, float( "inf" ) ];
            elif direction == 3: #move up
                nextx = x;
                nexty = y-1;
                if( y < 0 ):
                    break;
                req = [1,0,1,0];

                next_pos = self.xy_index_conversion( [nextx,nexty] );
                if( next_pos == goal ):
                    x = nextx;
                    y = nexty;
                    distance = distance + 1;
                    break;

                next_set = self[ next_pos ];
                if( next_set == [1,0,1,1] ):
                    return [ nextx, nexty, float( "inf" ) ];
            else:
                break;

            x = nextx;
            y = nexty;
            distance = distance + 1;

            if( next_set != req ):
                keep_moving = 0;

        values = [x, y, distance];
        return values;

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
        if key.get_pressed()[K_SPACE]:
            import time;
            total_time = 0;
            total_CR = 0;

            for i in range(0,100):
                X = labyrinthe((50,50));
                test_start = random.randrange(len(X))
                test_exit = random.randrange(len(X))

                start_time = time.time();
                test_path = X.get_path( test_start, test_exit );
                stop_time = time.time();
                time_diff = stop_time - start_time;

                distance_traveled = len( test_path );
                start_coord = X.index_xy_conversion( test_start );
                exit_coord  = X.index_xy_conversion( test_exit  );
                distance = X.distance_between_coords( start_coord, exit_coord );
                CR = distance_traveled / distance;

                total_time = total_time + time_diff;
                total_CR = total_CR + CR;

            average_time = total_time / 100.0;
            average_CR = total_CR / 100.0;
            print average_time;
            print total_time;
