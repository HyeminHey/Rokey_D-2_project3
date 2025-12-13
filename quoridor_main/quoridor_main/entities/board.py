from typing import Set, List, Union
import pygame

from quoridor_main.helpers import log
from quoridor_main.network.server import EnhancedServer, Functions
import quoridor_main.config as cfg

from quoridor_main.ai.action import ActionMovePawn, ActionPlaceWall
from quoridor_main.ai.ai import AI

from quoridor_main.entities.drawable import Drawable
from quoridor_main.entities.pawn import Pawn
from quoridor_main.entities.cell import Cell
from quoridor_main.entities.wall import Wall
from quoridor_main.entities.coord import Coord

from quoridor_main.config import DIR


class Board(Drawable):
    """ Quoridor board.
    This object contains te state of the game.
    """

    def __init__(self,
                 screen: pygame.Surface,
                 rows=cfg.DEF_ROWS,
                 cols=cfg.DEF_COLS,
                 cell_padding=cfg.CELL_PAD,
                 color=cfg.BOARD_BG_COLOR,
                 border_color=cfg.BOARD_BRD_COLOR,
                 border_size=cfg.BOARD_BRD_SIZE):

        Drawable.__init__(self, screen=screen, color=color, border_color=border_color, border_size=border_size)
        self.rows: int = rows
        self.cols: int = cols
        self.cell_pad = cell_padding
        self.mouse_wall = None  # Wall painted on mouse move
        self.player: int = 0  # Current player 0 or 1 -> 첫 시작이 사람!
        self.board: List[List[Cell]] = []
        self.computing = False  # True if a non-human player is moving
        self._state = None
        self.won_player: int = None

        # Create NETWORK server
        try:
            if cfg.NETWORK_ENABLED:
                self.server = EnhancedServer(("localhost", cfg.PORT))
                log('Network server active at TCP PORT ' + str(cfg.PORT))
                self.server.register_introspection_functions()
                self.server.register_instance(Functions())
                self.server.start()
        except BaseException:
            log('Could not start network server')
            self.server = None

        for i in range(rows):
            self.board.append([])
            for j in range(cols):
                self.board[-1].append(Cell(screen, self, coord=Coord(i, j)))

        # Pawn 2개 생성, 리스트 형식으로 들어가 있음
        self.pawns: List[Pawn] = []
        self.pawns += [Pawn(screen=screen,
                            board=self,
                            color=cfg.PAWN_A_COL,
                            border_color=cfg.PAWN_BORDER_COL,
                            coord=Coord(rows - 1, cols >> 1),  # Centered
                            # URL = SERVER_URL + ':%i' % (BASE_PORT + PAWNS)
                            )]
        self.pawns += [Pawn(screen=screen,
                            board=self,
                            color=cfg.PAWN_B_COL,
                            border_color=cfg.PAWN_BORDER_COL,
                            coord=Coord(0, col=cols >> 1)  # Centered
                            )]

        self.regenerate_board(cfg.CELL_COLOR, cfg.CELL_BORDER_COLOR)
        self.num_players = cfg.DEFAULT_NUM_PLAYERS
        self.walls: Set[Wall] = set()  # Walls placed on board
        self.draw_players_info()
        self._AI = []
        self._AI += [AI(self.pawns[1], level=cfg.LEVEL)]


    def reset_AI(self):
        # self._AI += [AI(self.pawns[0])]
        self._AI[0] = [AI(self.pawns[1], level=cfg.LEVEL)]

    def regenerate_board(self, c_color, cb_color, c_width=cfg.CELL_WIDTH, c_height=cfg.CELL_HEIGHT):
        """ Regenerate board colors and get_cell positions.
        Must be called on initialization or whenever a screen attribute
        changes (eg. color, board size, etc)
        """
        y = self.cell_pad
        for i in range(self.rows):
            x = self.cell_pad
            for j in range(self.cols):
                cell = self.board[i][j]
                cell.x, cell.y = x, y
                cell.color = c_color
                cell.border_color = cb_color
                cell.height = c_height
                cell.width = c_width
                cell.pawn = None
                x += c_width + self.cell_pad
            y += c_height + self.cell_pad

        for pawn in self.pawns:
            pawn.cell = self.get_cell(pawn.coord)

    def draw(self):
        """ Draws a squared n x n board, defaults
        to the standard 9 x 9
        """
        super().draw()

        #셀 그리는 코드
        for row in self:
            for cell in row:
                cell.draw()

        if cfg.__DEBUG__:
            for p in self.pawns:
                if p.AI:
                    p.distances.draw()
                    break
        #벽 그리는 코드
        for wall in self.walls:
            wall.draw()

    def get_cell(self, coord: Coord) -> Cell:
        """ Returns board get_cell at the given the coord
        """
        return self.board[coord.row][coord.col]

    def set_cell(self, coord: Coord, value: Cell):
        """ Updates a cell in the board with the new Cell
        instance at the given coord
        """
        self.board[coord.row][coord.col] = value

    def __getitem__(self, i: int) -> List[Cell]:
        return self.board[i]

    def in_range(self, coord: Coord) -> bool:
        """ Returns whether te given coordinate are within the board or not
        """
        return 0 <= coord.col < self.cols and 0 <= coord.row < self.rows

    def putWall(self, wall: Wall) -> None:
        """ Puts the given wall on the board.
        The cells are updated accordingly
        """
        if wall in self.walls:
            return  # If already put, nothing to do

        self.walls.add(wall)
        i, j = wall.coord

        if wall.horiz:
            self.board[i][j].set_path(DIR.S, False)
            self.board[i][j + 1].set_path(DIR.S, False)
        else:
            self.board[i][j].set_path(DIR.W, False)
            self.board[i + 1][j].set_path(DIR.W, False)

        self._state = None
    
    def removeWall(self, wall: Wall) -> None:
        """ Removes a wall from the board.
        The cells are updated accordingly
        """
        if wall not in self.walls:
            return  # Already removed, nothing to do

        self.walls.remove(wall)
        i, j = wall.coord

        if wall.horiz:
            self.board[i][j].set_path(DIR.S, True)
            self.board[i][j + 1].set_path(DIR.S, True)
        else:
            self.board[i][j].set_path(DIR.W, True)
            self.board[i + 1][j].set_path(DIR.W, True)

        self._state = None


    
    # # 필요 없는 코드
    # def onMouseClick(self, x, y):
    #     """ Dispatch mouse click Event
    #     """
    #     cell = self.which_cell(x, y) # 셀 선택
    #     if cell is not None: # 셀 선택이 되었다면
    #         pawn = self.current_player # 폰을 현재 플레이어로 설정
    #         if not pawn.can_move(cell.coord): # 폰이 셀의 좌표로 움직일 수 없다면 리턴
    #             return                

    #         self.do_action(ActionMovePawn(pawn.coord, cell.coord)) # 폰이 셀의 좌표로 움직일 수 있다면 doaction / from,to전달
    #         cell.set_focus(False) # 마우스 하이라이트 끔
    #         self.draw() # 보드 전체 다시 draw

    #         if self.finished:
    #             self.draw_player_info(self.player)
    #             return

    #         self.next_player() # 플레이어 변경
    #         self.draw_players_info() # 폰 그리고 리턴
    #         return

    #     wall = self.wall(x, y) # wall로 인자 설정하고 new_wall로 Wall 객체 반환하게 됨
    #     if not wall:
    #         return

    #     if self.can_put_wall(wall): # 벽을 놓을 수 있는지 판단하고 putWall (self.walls에 저장됨)
    #         self.do_action(ActionPlaceWall(wall))
    #         self.next_player()
    #         self.draw_players_info()

    # def onMouseMotion(self, x, y):
    #     """ Get mouse motion event and acts accordingly
    #     """
    #     if not self.rect.collidepoint(x, y):
    #         return

    #     for row in self.board:
    #         for cell in row:
    #             cell.onMouseMotion(x, y)

    #     if self.which_cell(x, y):
    #         if self.mouse_wall:
    #             self.mouse_wall = None
    #             self.draw()

    #         return  # The focus was on a get_cell, we're done

    #     if not self.current_player.walls:
    #         return  # The current player has run out of walls. We're done

    #     wall = self.wall(x, y)
    #     if not wall:
    #         return

    #     if self.can_put_wall(wall):
    #         self.mouse_wall = wall
    #         self.draw()
    #         wall.draw()

    def apply_player_action(self, req_list):

        ##변환하는 함수 boardrosnode에 있는거 가져오기
        t = req_list[0]
        r = req_list[1]
        c = req_list[2]
        
        if t == 1:
            cell = Cell(self.screen, self, coord=Coord(r, c))
            if cell is not None:
                pawn = self.current_player
                if not pawn.can_move(cell.coord):
                    log("pawn can't move to that coord")
                    return False
            
            self.do_action(ActionMovePawn(pawn.coord, cell.coord))
            self.draw()
            
            # if self.finished:
            #     self.draw_player_info(self.player)
            #     if self.won_player == 0:
            #         return "player"
            #     elif self.won_player == 1:
            #         return "AI"
            if self.finished:
                self.draw_player_info(self.player)
                log("finished")
                # reset하는 함수 만들어서 그거 트리거로 써야할까?
                return None

                
            self.next_player()
            self.draw_players_info()
            # 만약 다음이 AI라면 AI 턴 실행 
            if self.current_player.AI:
                self.computing = True
                self.computer_move()  
                return True
            return False
        
        if abs(t) == 2:
            if t == 2:
                horiz = False
            elif t == -2:
                horiz = True
            wall = self.new_wall(Coord(r, c), horiz)
            # log(f"can_put_wall? = {self.can_put_wall(wall)}")
            if not wall:
                return False
            avail = self.can_put_wall(wall)
            if avail:
                self.do_action(ActionPlaceWall(wall))
                if self.finished:
                    self.draw_player_info(self.player)
                    log("finished")
                    return
                self.next_player()
                self.draw_players_info()
                # 만약 다음이 AI라면 AI 턴 실행 
                if self.current_player.AI:
                    self.computing = True
                    self.computer_move()  
                    return True
                return False
            elif avail == False:
                log("can't put wall")
                return None


    def can_put_wall(self, wall) -> bool:
        """ Returns whether the given wall can be put
        on the board.
        """
        if not self.current_player.walls:
            return False

        # Check if any wall has already got that place...
        for w in self.walls:
            if wall.collides(w):
                return False

        result = True
        self.putWall(wall)

        for pawn in self.pawns:
            if not pawn.can_reach_goal():
                result = False
                break

        self.removeWall(wall)
        return result

    def wall(self, x, y) -> Union[Wall, None]:
        """ Factory which returns which wall is below mouse cursor at x, y coords.
        Returns None if no wall matches x, y coords
        """
        if not self.rect.collidepoint(x, y):
            return None

        # Wall: Guess which top-left get_cell is it 가장 가까운 셀 찾기
        j = (x - self.x) // (self.board[0][0].width + self.cell_pad)
        i = (y - self.y) // (self.board[0][0].height + self.cell_pad)
        cell = self.board[i][j]

        # Wall: Guess if it is horizontal or vertical
        horiz = x < (cell.x + cell.width)
        max_idx = self.rows - 2
        if horiz:
            if j > max_idx:
                j = max_idx
        else:
            if i > max_idx:
                i = max_idx

        if i > max_idx or j > max_idx:
            return None

        return self.new_wall(Coord(i, j), horiz, cell.wall_color)

    def new_wall(self, coord: Coord, horiz: bool, color: pygame.Color = None) -> Wall:
        """ Wall factory. Creates a new wall
        """
        if color is None:
            color = self.board[0][0].wall_color
        return Wall(self.screen, self, color, coord, horiz)

    @property
    def x(self):
        """ Absolute left coordinate
        """
        return self.board[0][0].x

    @property
    def y(self):
        """ Absolute left coordinate
        """
        return self.board[0][0].y

    @property
    def width(self):
        return (self.cell_pad + self.board[0][0].width) * self.cols

    @property
    def height(self):
        return (self.cell_pad + self.board[0][0].height) * self.rows

    @property
    def rect(self):
        return pygame.Rect(self.x, self.y, self.width, self.height)

    def next_player(self):
        """ Switches to next player
        """
        self.player = (self.player + 1) % self.num_players
        self.update_pawns_distances()

    def update_pawns_distances(self):
        for pawn in self.pawns:
            pawn.distances.update()

    def which_cell(self, x, y):
        """ Returns an instance of the get_cell for which (x, y) screen coord
        matches. Otherwise, returns None if no get_cell is at (x, y) screen
        coords.
        """
        for row in self.board:
            for cell in row:
                if cell.rect.collidepoint(x, y):
                    return cell

        return None

    @property
    def current_player(self):
        """ Returns current player's pawn
        """
        return self.pawns[self.player]

    def draw_player_info(self, player_num):
        """ Draws player pawn at board + padding_offset
        """
        pawn = self.pawns[player_num]
        r = pawn.rect
        r.x = self.rect.x + self.rect.width + cfg.PAWN_PADDING
        r.y = (player_num + 1) * (r.height + cfg.PAWN_PADDING)
        if self.current_player is pawn:
            pygame.draw.rect(self.screen, cfg.CELL_VALID_COLOR, r, 0)
            pygame.draw.rect(self.screen, pawn.border_color, r, 2)
        else:
            pygame.draw.rect(self.screen, self.color, r, 0)

        pawn.draw(r)
        rect = pygame.Rect(r.x + 1, r.y + r.h + 3, cfg.GAUGE_WIDTH, cfg.GAUGE_HEIGHT)

        if pawn.percent is not None:
            pygame.draw.rect(self.screen, cfg.FONT_BG_COLOR, rect, 0)  # Erases old gauge bar
            rect.width = int(cfg.GAUGE_WIDTH * pawn.percent)
            pygame.draw.rect(self.screen, cfg.GAUGE_COLOR, rect, 0)
            rect.width = cfg.GAUGE_WIDTH
            pygame.draw.rect(self.screen, cfg.GAUGE_BORDER_COLOR, rect, 1)
        else:
            pygame.draw.rect(self.screen, cfg.FONT_BG_COLOR, rect, 0)

        r.x += r.width + 20
        r.width = 6
        pygame.draw.rect(self.screen, cfg.WALL_COLOR, r, 0)

        r.x += r.width * 2 + 10
        r.y += r.height // 2 - 5
        r.height = cfg.FONT_SIZE
        r.width *= 3
        pygame.draw.rect(self.screen, cfg.FONT_BG_COLOR, r, 0)  # Erases previous number
        self.msg(r.x, r.y, str(pawn.walls))

        if self.finished and self.current_player == pawn:
            self.msg(r.x + cfg.PAWN_PADDING, r.y, "PLAYER %i WINS!" % (1 + self.player))
            x = self.rect.x
            y = self.rect.y + self.rect.height + cfg.PAWN_PADDING
            self.msg(x, y, "Press any key to EXIT")
            #log 추가
            log(f"player {self.player} win! game finished")
            self.won_player = self.player
            log(f"player{self.won_player} won this game!")

    def msg(self, x, y, str_, color=cfg.FONT_COLOR, fsize=cfg.FONT_SIZE):
        font = pygame.font.SysFont(None, fsize)
        fnt = font.render(str_, True, color)
        self.screen.blit(fnt, (x, y))

    def draw_players_info(self):
        """ Calls the above function for every player.
        """
        for i in range(len(self.pawns)):
            self.draw_player_info(i)

    def do_action(self, action: Union[ActionPlaceWall, ActionMovePawn]):
        """ Performs a playing action: move a pawn or place a barrier.
        Transmit the action to the network, to inform other players.
        """
        player_id = self.current_player.id

        if isinstance(action, ActionPlaceWall):
            wdir = 'horizontal' if action.horiz else 'vertical'
            log('Player %i places %s wall at (%i, %i)' % (player_id, wdir, action.coord.row, action.coord.col))
            self.putWall(self.new_wall(action.coord, action.horiz))
            self.current_player.walls -= 1
        else:
            log('Player %i moves to (%i, %i)' % (player_id, action.dest.row, action.dest.col))
            self.current_player.move_to(action.dest)
            self._state = None

        for pawn in self.pawns:
            if pawn.is_network_player:
                pawn.NETWORK.do_action(action)

    def computer_move(self):
        """ Performs computer moves for every non-human player
        """
        while self.current_player.AI and not self.finished:
            self.draw()
            self.draw_players_info()
            action, x = self.current_player.AI.move()
            # pygame.mixer.music.load('./media/chime.ogg')
            # pygame.mixer.music.load('quoridor_al/media/chime.ogg')
            # pygame.mixer.music.play()
            self.ai_action = action
            self.do_action(action)

            if self.finished:
                break

            self.next_player()

        self.draw()
        self.draw_players_info()
        self.computing = False

    @property
    def finished(self):
        """ Returns whether the match has finished or not.
        """
        return any(pawn.coord in pawn.goals for pawn in self.pawns)

    @property
    def state(self):
        """ Status serialization in a t-uple
        """
        if self._state is not None:
            return self._state

        result = str(self.player)  # current player
        result += ''.join(p.state for p in self.pawns)
        result += ''.join(self.board[i][j].state for j in range(self.cols - 1) for i in range(self.rows - 1))
        self._state = result

        return result
