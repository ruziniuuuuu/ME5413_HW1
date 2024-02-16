from matplotlib import pyplot as plt

def plot_map(road_polylines):
    ax = plt.gca()
    fig = plt.gcf()
    fig.set_size_inches(12, 12)
    fig.set_facecolor('xkcd:grey') 
    ax.set_facecolor('xkcd:grey')
    ax.axis([600, 720, -2470, -2350])
    for polyline in road_polylines:
        map_type = polyline[0,6]
        if map_type == 6:
            plt.plot(polyline[:, 0], polyline[:, 1], 'w', linestyle='dashed', linewidth=1)
        elif map_type == 7:
            plt.plot(polyline[:, 0], polyline[:, 1], 'w', linestyle='solid', linewidth=1)
        elif map_type == 8:
            plt.plot(polyline[:, 0], polyline[:, 1], 'w', linestyle='solid', linewidth=1)
        elif map_type == 9:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='dashed', linewidth=1)
        elif map_type == 10:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='dashed', linewidth=1)
        elif map_type == 11:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='solid', linewidth=1)
        elif map_type == 12:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='solid', linewidth=1)
        elif map_type == 13:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='dotted', linewidth=1)
        elif map_type == 15:
            plt.plot(polyline[:, 0], polyline[:, 1], 'k', linewidth=1)
        elif map_type == 16:
            plt.plot(polyline[:, 0], polyline[:, 1], 'k', linewidth=1)
    return ax

def plot_map_single(road_polylines, region):
    ax = plt.gca()
    fig = plt.gcf()
    fig.set_size_inches(5, 5)
    fig.set_facecolor('xkcd:white') 
    ax.set_facecolor('xkcd:white')
    ax.axis(region)
    for polyline in road_polylines:
        map_type = polyline[0,6]
        if map_type == 6:
            plt.plot(polyline[:, 0], polyline[:, 1], 'grey', linestyle='dashed', linewidth=1)
        elif map_type == 7:
            plt.plot(polyline[:, 0], polyline[:, 1], 'grey', linestyle='solid', linewidth=1)
        elif map_type == 8:
            plt.plot(polyline[:, 0], polyline[:, 1], 'grey', linestyle='solid', linewidth=1)
        elif map_type == 9:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='dashed', linewidth=1)
        elif map_type == 10:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='dashed', linewidth=1)
        elif map_type == 11:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='solid', linewidth=1)
        elif map_type == 12:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='solid', linewidth=1)
        elif map_type == 13:
            plt.plot(polyline[:, 0], polyline[:, 1], 'xkcd:yellow', linestyle='dotted', linewidth=1)
        elif map_type == 15:
            plt.plot(polyline[:, 0], polyline[:, 1], 'k', linewidth=1)
        elif map_type == 16:
            plt.plot(polyline[:, 0], polyline[:, 1], 'k', linewidth=1)
    return ax