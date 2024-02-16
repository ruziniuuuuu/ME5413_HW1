def save_detections(detections, output_file):
    """
    Saves the list of detections to a text file.

    :param detections: List of tuples, where each tuple contains the coordinates
                       (x, y, width, height) of the detected target.
    :param output_file: Path to the output text file.
    """
    with open(output_file, 'w') as file:
        for detection in detections:
            # Format the detection as x,y,width,height
            detection_str = " {},{},{},{}\n".format(*detection)
            file.write(detection_str)