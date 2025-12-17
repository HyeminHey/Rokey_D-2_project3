# -*- coding: utf-8 -*-

import os
from pygame import Color

from quoridor_main.entities.coord import Coord

__doc__ = """ Centralizes all global configuration flags """

# Debug FLAG
__DEBUG__ = False

# Frame rate
FRAMERATE = 25

# Config Options
GAME_TITLE = 'Quoridor Rokey D-2'
DEFAULT_NUM_PLAYERS = 2

# Cell size
# CELL_WIDTH = 50
# CELL_HEIGHT = 50
# CELL_PAD = 7
# CELL_BORDER_SIZE = 2
CELL_WIDTH = 70
CELL_HEIGHT = 70
CELL_PAD = 10 # 장벽 놓는 곳 너비
CELL_BORDER_SIZE = 2 # 테두리 두께

# Default Number of rows and cols
DEF_ROWS = 7
DEF_COLS = 7

# Number of Walls per player
NUM_WALLS = 6

# Messages Font
WHO_SIZE = 50
RULE_SIZE = 22

# Pawn size
PAWN_WIDTH = CELL_WIDTH - 15
PAWN_HEIGHT = CELL_HEIGHT - 15

### COLORS ###
# Font Color & SIZE
FONT_COLOR = Color(255, 255, 255)
FONT_BG_COLOR = Color(75, 75, 75) 
FONT_SIZE = 30 #16

# Board Background and Border color and look
BOARD_BG_COLOR = Color(93, 0, 0) # Color(240, 255, 255)
BOARD_BRD_COLOR = Color(0, 0, 40) # 글씨 쓸 때 배경색?
BOARD_BRD_SIZE = 1

# Cell colors
CELL_BORDER_COLOR = Color(0, 0, 0) # Color(40, 40, 40)
CELL_COLOR = Color(0, 0, 10) # Color(120, 90, 60)
CELL_VALID_COLOR = Color(255, 255, 255) # Color(40, 120, 120)  # Cyan

# Wall Color
WALL_COLOR = Color(201, 174, 0) # Color(0, 0, 0)

# Pawns color
PAWN_A_COL = Color(147, 0, 0) # Color(158, 60, 60)  # Red
PAWN_B_COL = Color(103, 152, 253) # Color(60, 60, 158)  # Blue
PAWN_BORDER_COL = Color(0, 0, 0) # Color(188, 188, 80)  # Yellow

# Gauge bars
GAUGE_WIDTH = CELL_WIDTH * 3
GAUGE_HEIGHT = 15
GAUGE_COLOR = Color(21, 21, 21)
GAUGE_BORDER_COLOR = Color(0, 0, 0)

# Other constants
PAWN_PADDING = 50 # 25  # Pixels right to the board

# State Box
STATE_BOX_COLOR = Color(53, 53, 53)
STATE_BOX_FONT_SIZE = 100
class DIR:
    """ Directions
    """
    N = 0
    S = 1
    E = 2
    W = 3


DIRS = {DIR.N, DIR.S, DIR.E, DIR.W}  # Available directions
OPPOSITE_DIRS = [DIR.S, DIR.N, DIR.W, DIR.E]  # Reverse direction

# Delta to add to position to move into that direction
DIRS_DELTA = [Coord(-1, 0), Coord(+1, 0), Coord(0, -1), Coord(0, +1)]

# Network port
NETWORK_ENABLED = False  # Set to true to enable network playing
PORT = 8001  # This client port
BASE_PORT = 8000
SERVER_ADDR = 'localhost'
SERVER_URL = 'http://{}:{}'.format(SERVER_ADDR, PORT)

# Default AI playing level
LEVEL = 0

def set_level(level=0):
    global LEVEL
    LEVEL = level

# Infinite
INF = 99

# Cache
CACHE_ENABLED = False
CACHE_DIR = './__cache'
CACHE_AI_FNAME = os.path.join(CACHE_DIR, 'ai.memo')
CACHE_DIST_FNAME = os.path.join(CACHE_DIR, 'dist.memo')
