from typing import Optional, Tuple, Set, Any
from datetime import datetime

import pygame
import struct
import moderngl

class OpenGLRenderer:
    def __init__(self, 
                 size: Tuple[int,int],
                 shader_name: str,
                 display_check_matrix: bool = False,
                 distortion: Optional[float] = None,
                 is_shader_control_by_mouse: bool = False,
                 is_reversed: bool = False):
        screen = pygame.display.set_mode(size, pygame.constants.DOUBLEBUF | pygame.constants.OPENGL | pygame.constants.NOFRAME ).convert((0xff, 0xff00, 0xff0000, 0))
        ctx = moderngl.create_context()

        self._program = ctx.program(
            vertex_shader = open(f'./srunner/scenarios/emirror_overlay/shaders/{shader_name}.vert').read(),
            fragment_shader = open(f'./srunner/scenarios/emirror_overlay/shaders/{shader_name}.frag').read()
        )
        
        texture_coordinates = [0, 1,  1, 1,  0, 0,  1, 0]
        world_coordinates = [-1, -1,  1, -1,  -1,  1,  1,  1]
        render_indices = [0, 1, 2,  1, 2, 3]
                  
        vbo = ctx.buffer(struct.pack('8f', *world_coordinates))
        uvmap = ctx.buffer(struct.pack('8f', *texture_coordinates))
        ibo = ctx.buffer(struct.pack('6I', *render_indices))

        vao_content = [
            (vbo, '2f', 'in_coords'),
            (uvmap, '2f', 'in_uv')
        ]

        self._vao = ctx.vertex_array(self._program, vao_content, ibo) # pyright: ignore

        screen_texture = ctx.texture(size, 3, pygame.image.tostring(screen, 'RGB'))
        screen_texture.repeat_x = False
        screen_texture.repeat_y = False
        self._screen_texture = screen_texture

        self._zoom = 1.4
        self._convex_radius = distortion or -1.0
        self._timestamp = datetime.now().timestamp()

        self.screen = screen
        self.mouse = 0.0, 0.0

        self._is_shader_control_by_mouse = is_shader_control_by_mouse
        
        #self.ctx = ctx

        self._glsl_uniforms: Set[str] = set()
        self._inject_uniforms(size, display_check_matrix, is_reversed)
        
    def inject_uniforms(self, **kwargs: Any) -> None:
        for u in kwargs:
            uniform_name = f'u_{u}'
            if (uniform_name in self._glsl_uniforms):
                self._program[uniform_name] = kwargs[u]
        
    def zoomIn(self) -> None:
        self._zoom += 0.1
        self._convex_radius -= 0.25

    def zoom_out(self) -> None:
        self._zoom -= 0.1
        self._convex_radius += 0.25

    def render(self, texture_data: Any) -> None:
        if self._is_shader_control_by_mouse:
            if ('u_time' in self._glsl_uniforms):
                self._program['u_time'] = datetime.now().timestamp() - self._timestamp
            if ('u_mouse' in self._glsl_uniforms):
                self._program['u_mouse'] = self.mouse
            if ('u_zoom' in self._glsl_uniforms):
                self._program['u_zoom'] = self._zoom
            if ('u_convex_radius' in self._glsl_uniforms):
                self._program['u_convex_radius'] = self._convex_radius if self._convex_radius > 0.0 else 0.0
        
        self._screen_texture.write(texture_data)
        self._screen_texture.use()
        self._vao.render()
        
    # Internal

    def _inject_uniforms(self,
                       size: Tuple[int,int],
                       colorize: bool,
                       is_reversed: bool) -> None:
        # only for debugging
        for name in self._program:
            member = self._program[name]
            if isinstance(member, moderngl.Uniform):
                self._glsl_uniforms.add(name)
                print(f'OGL: {member.location}: uniform [{member.dimension}] {name}')
            elif isinstance(member, moderngl.Attribute):
                print(f'OGL: {member.location}: in [{member.dimension}] {name}')
        
        if self._is_shader_control_by_mouse:
            if ('u_resolution' in self._glsl_uniforms):
                self._program['u_resolution'] = size
            if ('u_colorize' in self._glsl_uniforms):
                self._program['u_colorize'] = colorize
                
        if ('u_convex_radius' in self._glsl_uniforms):
            self._program['u_convex_radius'] = self._convex_radius if self._convex_radius > 0.0 else 0.0
        if ('u_reversed' in self._glsl_uniforms):
            self._program['u_reversed'] = is_reversed
        