import math
from vex import *
from random import uniform, choice

brain = Brain()
controller = Controller(PRIMARY)

# Constants
FPS = 30
SCREEN_WIDTH, SCREEN_HEIGHT = 480, 240

# Ball variables
BALL_COLOR = Color.GREEN
BALL_START_VELOCITY_RANGE = [2, 3]
BALL_START_VELOCITY_X_BOOST = 1
BALL_RADIUS = 6
BALL_WALL_BOUNCE_FRICTION = 1.05
BALL_PADDLE_BOUNCE_SPEED = 1.25  # gets more speed!
BALL_JITTER = 0.05  # Velocity jitter
BALL_MIN_VEL_X = 2

# Paddle variables
PADDLE_COLOR = Color.WHITE
PADDLE_DISTANCE_TO_WALL = 20
PADDLE_WIDTH = 8
PADDLE_HEIGHT = 28
PADDLE_SPEED = 0.008
PADDLE_FRICTION = 0.94

# Environment variables
LINE_COLOR = Color.YELLOW
left_score = 0
right_score = 0
paddle_bounces = 1


def random_ball_velocity():
    return (uniform(BALL_START_VELOCITY_RANGE[0], BALL_START_VELOCITY_RANGE[1]) + BALL_START_VELOCITY_X_BOOST) * choice([-1, 1])


def random_ball_jitter():
    return uniform(-BALL_JITTER, BALL_JITTER)


def circle_rect_collision(circle_center, circle_radius, circle_velocity, rect_position, rect_width, rect_height, rect_velocity):
    global paddle_bounces
    
    relative_velocity_x = circle_velocity[0] - rect_velocity[0]
    relative_velocity_y = circle_velocity[1] - rect_velocity[1]
    
    closest_x = max(rect_position[0], min(circle_center[0], rect_position[0] + rect_width))
    closest_y = max(rect_position[1], min(circle_center[1], rect_position[1] + rect_height))

    dist_x = circle_center[0] - closest_x
    dist_y = circle_center[1] - closest_y
    dist_squared = dist_x ** 2 + dist_y ** 2

    if dist_squared <= circle_radius ** 2:
        if dist_squared == 0:
            # Handle zero-division error by reflecting velocity
            return -circle_velocity[0] + random_ball_jitter(), -circle_velocity[1] + random_ball_jitter()
        else:
            controller.rumble(".")

            # Collision occurred, calculate the new velocity by reflecting the old one
            dot_product = (relative_velocity_x * dist_x + relative_velocity_y * dist_y) / dist_squared
            new_relative_velocity_x = relative_velocity_x - 2 * dot_product * dist_x
            new_relative_velocity_y = relative_velocity_y - 2 * dot_product * dist_y

            # Add back the rect's velocity to get the new circle velocity
            new_velocity_x = new_relative_velocity_x + rect_velocity[0]
            new_velocity_y = new_relative_velocity_y + rect_velocity[1]

            paddle_bounces += 1
            
            return (new_velocity_x + random_ball_jitter(), new_velocity_y + random_ball_jitter())
    else:
        # No collision, return the original velocity
        return circle_velocity


class Ball:
    def __init__(self, x, y, radius, color, vel_x=0.0, vel_y=0.0, wall_bounce_friction=0.95, paddle_bounce_speed=1.1):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.wall_bounce_friction = wall_bounce_friction
        self.paddle_bounce_speed = paddle_bounce_speed

    def update(self, left_paddle, right_paddle):
        global left_score, right_score, paddle_bounces

        # Velocities
        self.x += self.vel_x
        self.y += self.vel_y

        # Min velocity
        if abs(self.vel_x) < BALL_MIN_VEL_X:  # Adjust this threshold as needed
            self.vel_x = (BALL_MIN_VEL_X * (1 if self.vel_x >= 0 else -1) * BALL_PADDLE_BOUNCE_SPEED ** paddle_bounces + self.vel_x * 4) / 5
        
        # Wall bouncing
        if self.y < self.radius:
            self.y = self.radius
            self.vel_x *= self.wall_bounce_friction
            self.vel_y *= -self.wall_bounce_friction
        elif self.y > SCREEN_HEIGHT - self.radius:
            self.y = SCREEN_HEIGHT - self.radius
            self.vel_x *= self.wall_bounce_friction
            self.vel_y *= -self.wall_bounce_friction
        
        # Win
        if self.x <= -self.radius:
            right_score += 1
            controller.rumble(".-.")
            self.x, self.y = SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2
            self.vel_x, self.vel_y = random_ball_velocity(), random_ball_velocity()
            paddle_bounces = 1
        elif self.x >= self.radius + SCREEN_WIDTH:
            left_score += 1
            controller.rumble(".-.")
            self.x, self.y = SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2
            self.vel_x, self.vel_y = random_ball_velocity(), random_ball_velocity()
            paddle_bounces = 1
        
        # Paddle collision
        if self.x < PADDLE_DISTANCE_TO_WALL + PADDLE_WIDTH * 2:
            self.vel_x, self.vel_y = circle_rect_collision([self.x, self.y], self.radius, [self.vel_x, self.vel_y], [left_paddle.x, left_paddle.y], left_paddle.width, left_paddle.height, [0, left_paddle.vel_y])
        elif self.x > SCREEN_WIDTH - PADDLE_DISTANCE_TO_WALL - PADDLE_WIDTH * 2:
            self.vel_x, self.vel_y = circle_rect_collision([self.x, self.y], self.radius, [self.vel_x, self.vel_y], [right_paddle.x, right_paddle.y], right_paddle.width, right_paddle.height, [0, left_paddle.vel_y])

    def draw(self):
        brain.screen.set_fill_color(self.color)
        brain.screen.draw_circle(self.x, self.y, self.radius)


class Paddle:
    def __init__(self, x, y, width, height, color, friction, vel_y=0):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        self.friction = friction
        self.vel_y = vel_y

    def update(self, vel_y):
        # Velocity stuff
        self.vel_y += vel_y
        self.vel_y *= self.friction
        self.y += self.vel_y

        # Bounds checking
        if self.y < 0:
            self.y = 0
            self.vel_y = 0
        elif self.y > SCREEN_HEIGHT - PADDLE_HEIGHT:
            self.y = SCREEN_HEIGHT - PADDLE_HEIGHT
            self.vel_y = 0
    
    def draw(self):
        brain.screen.set_fill_color(self.color)
        brain.screen.draw_rectangle(self.x, self.y, self.width, self.height)


# Variables
ball = Ball(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, BALL_RADIUS, BALL_COLOR, random_ball_velocity(), random_ball_velocity(), BALL_WALL_BOUNCE_FRICTION, BALL_PADDLE_BOUNCE_SPEED)
left_paddle = Paddle(PADDLE_DISTANCE_TO_WALL, SCREEN_HEIGHT / 2 - PADDLE_HEIGHT / 2, PADDLE_WIDTH, PADDLE_HEIGHT, PADDLE_COLOR, PADDLE_FRICTION)
right_paddle = Paddle(SCREEN_WIDTH - PADDLE_DISTANCE_TO_WALL - PADDLE_WIDTH, SCREEN_HEIGHT / 2 - PADDLE_HEIGHT / 2, PADDLE_WIDTH, PADDLE_HEIGHT, PADDLE_COLOR, PADDLE_FRICTION)

brain.screen.set_font(FontType.MONO20)  # 10 rows x 20 cols
while True:
    # Buttons / inputs
    left_down = controller.buttonL1.pressing()
    left_up = controller.buttonL2.pressing()
    right_down = controller.buttonR1.pressing()
    right_up = controller.buttonR2.pressing()
    if abs(controller.axis3.position()) > 5:
        left_movement = controller.axis3.position() * -PADDLE_SPEED  # Y axis is inverted
    elif left_down or left_up:
        left_movement = PADDLE_SPEED * (left_down - left_up) * 100
    else:
        left_movement = 0
    if abs(controller.axis2.position()) > 5:
        right_movement = controller.axis2.position() * -PADDLE_SPEED  # Y axis is inverted
    elif right_down or right_up:
        right_movement = PADDLE_SPEED * (right_down - right_up) * 100
    else:
        right_movement = 0

    if left_score >= 5:
        brain.screen.set_pen_color(Color.RED)
        brain.screen.set_font(FontType.MONO20)
        brain.screen.set_cursor(5, 20)
        brain.screen.print("Left Wins!")
        controller.rumble("----.")
        break
    if right_score >= 5:
        brain.screen.set_pen_color(Color.RED)
        brain.screen.set_font(FontType.MONO20)
        brain.screen.set_cursor(5, 20)
        brain.screen.print("Right Wins!")
        controller.rumble("----.")
        break

    # Update before drawing
    left_paddle.update(left_movement)
    right_paddle.update(right_movement)
    ball.update(left_paddle, right_paddle)

    # To minimize draw screen lag, only nesscary code after clearing screen and finish drawing
    # Clear the screen, draw paddles, draw the ball, line in the middle, and the scores
    brain.screen.clear_screen(Color.BLACK)
    
    # Draw line in the middle
    brain.screen.set_pen_width(1)
    brain.screen.set_pen_color(LINE_COLOR)
    brain.screen.draw_line(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, SCREEN_HEIGHT)
    brain.screen.set_pen_width(0)

    # Draw paddles and balls
    left_paddle.draw()
    right_paddle.draw()
    ball.draw()

    # Draw score
    brain.screen.set_fill_color(Color.BLACK)
    brain.screen.set_pen_color(Color.WHITE)
    brain.screen.set_cursor(1, 12)  # adjust later
    brain.screen.print(left_score)
    brain.screen.set_cursor(1, 38)  # adjust later
    brain.screen.print(right_score)

    brain.screen.render()

    wait(1 / FPS, SECONDS)

# The game has ended, so render the win text
brain.screen.render()
