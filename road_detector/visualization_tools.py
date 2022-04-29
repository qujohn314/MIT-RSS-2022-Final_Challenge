from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class VisualizationTools:

    @staticmethod
    def plot_line(x, y, publisher, color = (1., 0., 0.), frame = "/base_link"):
        """
        Publishes the points (x, y) to publisher
        so they can be visualized in rviz as
        connected line segments.
        Args:
            x, y: The x and y values. These arrays
            must be of the same length.
            publisher: the publisher to publish to. The
            publisher must be of type Marker from the
            visualization_msgs.msg class.
            color: the RGB color of the plot.
            frame: the transformation frame to plot in.
        """
        # Construct a line
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.header.frame_id = frame

        # Set the size and color
        line_strip.scale.x = 0.1
        line_strip.scale.y = 0.1
        line_strip.color.a = 1.
        line_strip.color.r = color[0]
        line_strip.color.g = color[1]
        line_strip.color.g = color[2]

        # Fill the line with the desired values
        print(x)
        for xi, yi in zip(x, y):
            p = Point()
            p.x = xi
            p.y = yi
            line_strip.points.append(p)

        # Publish the line
        publisher.publish(line_strip)

    @staticmethod
    def plot_point(x, y, publisher, color=(0., 1., 0.), frame= "/base_link"):

        start_point_marker = Marker()
        start_point_marker.header.frame_id = frame
        start_point_marker.type = start_point_marker.SPHERE
        start_point_marker.action = start_point_marker.ADD
        start_point_marker.scale.x = 0.4
        start_point_marker.scale.y = 0.4
        start_point_marker.scale.z = 0.4
        
        start_point_marker.pose.position.x = x
        start_point_marker.pose.position.y = y
        start_point_marker.pose.position.z = 0

        # start_point_marker.pose.orientation = Quaternion(0,0,0,1)

        # start_point_marker.color.a = 0.5
        start_point_marker.color.r = color[0]
        start_point_marker.color.g = color[1]
        start_point_marker.color.b = color[2]

        # Publish the line
        publisher.publish(start_point_marker)