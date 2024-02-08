def load_firsttrack(firsttrack_txt):
    # Load the first frame's bounding box from file
    with open(firsttrack_txt, 'r') as file:
        line = file.readline().strip()
        x, y, w, h = map(int, line.split(','))
    return x, y, w, h

def load_tracks(gt_path):
    # Load ground truth data from file
    # Expected format: [ (x, y, w, h), ...] for each frame
    tracks = []
    with open(gt_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            tracks.append((int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])))
    return tracks
