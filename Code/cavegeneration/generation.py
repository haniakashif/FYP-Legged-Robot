# SPDX-License-Identifier: BSD-3-Clause

from graph import Graph
from display import Display
from algorithm import Algorithm
from config import Color, Config
import subprocess
import argparse
import datetime
import multiprocessing
from tools import Tools
import random as rd
import time
import json
import os
from lava_tubes import ProceduralLavaTube
import numpy as np

class Generator:

    def __init__(self, name_p, graph_path_p) -> None:       
        # Get the number of graphs
        self.nb_graphs = Config.NB_GENERATION.value
        self.visualization = Config.OPEN_VISUALIZATION.value
        self.graphs=[]

        self.starting_time = time.time()
        self.graph_path = []

        # Get the name of the current graph generation
        if name_p == None or '':
            self.name = Config.DEFAULT_NAME.value
        else:
            self.name = name_p
        
        if graph_path_p == None or '':
            self.graph_path = None
        else:
            # Check if mutli-node generation
            self.name = f'{graph_path_p}_regen_{datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}'
            graph_path_p = f'{os.getcwd()}/data/{graph_path_p}'
            dir_ = os.listdir(graph_path_p)
            if os.path.isdir(f'{graph_path_p}/{dir_[0]}'):
                # Multi-node generation
                for path in dir_:
                    self.graph_path.append(str(f'{graph_path_p}/{path}'))
            else:
                self.graph_path = graph_path_p


        if self.graph_path:
            # Regenerate graph
            if type(self.graph_path) == list:
                index = 0
                for path in self.graph_path:
                    if Config.GENERATE_GRAPH_IMAGE.value:
                        saving_path = os.getcwd()+'/data/'+self.name+'/'+ str(index)
                        self.create_graph_picture(path_p=path, saving_path_p=saving_path)
                    self.create_mesh(index, graph_path_p=path)
                    index+=1
                                
            else:
                if Config.GENERATE_GRAPH_IMAGE.value:
                        saving_path = os.getcwd()+'/data/'+self.name
                        self.create_graph_picture(path_p=path, saving_path_p=saving_path)
                self.create_mesh(0, graph_path_p=self.graph_path)

        else:
            # Start generation
            if Config.PARALLELIZATION.value:
                # Make n graphs in different CPU cores (Only for the graph generation)  
                list_process = [i for i in range(self.nb_graphs)]
                with multiprocessing.Pool(processes=self.nb_graphs) as pool:
                    result = pool.map(self.generator, list_process)
                # Create the mesh
                if Config.GENERATE_MESH.value:
                    for index in list_process:
                        path = f'{os.getcwd()}/data/{self.name}/{index}'
                        self.create_mesh(index, graph_path_p=path)
            
            else:
                for index in range(self.nb_graphs):
                    path = os.getcwd()+'/data/'+self.name+'/'+str(index)
                    self.generator(index, path)
                    if Config.GENERATE_MESH.value:
                        duration = self.create_mesh(index, graph_path_p=path)
                        


    def generator(self, index_p, path_p):
        """
        Main generation frame. Used for multiprocessing
        """
        index = index_p
        self.generate_graph(index)
        
        # Create picture for n graphs
        if Config.GENERATE_GRAPH_IMAGE.value:
                self.create_graph_picture(path_p = path_p, saving_path_p=path_p)


    # def generate_graph(self, index_p):
    #     """
    #     Main logic of the graph generation
    #     """


    #     index = index_p
    #     print(f"{Color.OKBLUE.value} == Graph {index} generation begins == {Color.ENDC.value}")
        
        
    #     tube = ProceduralLavaTube(
    #         shape=Config.GENERATION_SIZE.value,
    #         seed=Config.LT_SEED.value,
    #         tube_major_radius=Config.LT_TUBE_MAJOR_RADIUS.value,
    #         tube_minor_radius=Config.LT_TUBE_MINOR_RADIUS.value,
    #         min_neck_minor=Config.LT_NECK_MINOR_MIN.value,
    #         chamber_major=Config.LT_CHAMBER_MAJOR_SCALE.value,
    #         chamber_minor=Config.LT_CHAMBER_MINOR_SCALE.value,
    #         floor_level_ratio=Config.LT_FLOOR_LEVEL_RATIO.value
    #     )

    #     tube.generate_main_path(
    #         length=Config.LT_MAIN_PATH_LENGTH.value
    #     )

    #     tube.add_side_branches(
    #         n_branches=Config.LT_BRANCHES_N.value,
    #         min_length=Config.LT_BRANCHES_MIN_LENGTH.value,
    #         max_length=Config.LT_BRANCHES_MAX_LENGTH.value,
    #         min_offset=Config.LT_BRANCHES_MIN_OFFSET.value,
    #         max_offset=Config.LT_BRANCHES_MAX_OFFSET.value,
    #         p_rejoin=Config.LT_BRANCHES_P_REJOIN.value
    #     )

    #     tube.carve_tubes(
    #         n_chambers=Config.LT_CHAMBERS_N.value,
    #         n_necks=Config.LT_NECKS_N.value,
    #         shape_profiles=Config.LT_SHAPE_PROFILES.value,
    #         rect_min_w=Config.LT_RECT_MIN_WIDTH.value,
    #         rect_max_w=Config.LT_RECT_MAX_WIDTH.value,
    #         rect_min_h=Config.LT_RECT_MIN_HEIGHT.value,
    #         rect_max_h=Config.LT_RECT_MAX_HEIGHT.value,
    #         crawlspace_h=Config.LT_CRAWLSPACE_HEIGHT.value,
    #         crevasse_w=Config.LT_CREVASSE_WIDTH.value
    #     )

    #     # Also add the roughness method, controlled by the config
    #     tube.add_surface_roughness(
    #         amount=Config.LT_ROUGHNESS_AMOUNT.value,
    #         repeats=Config.LT_ROUGHNESS_REPEATS.value
    #     )

    #     tube.smooth(
    #         sigma=Config.LT_SMOOTH_SIGMA.value
    #     )


    #     graph = Graph(self.name, index, Config.MAX_CREATED_NODE_ON_CIRCLE.value)
    #     node_id = 0
    #     for name, path, major, minor in tube.paths:
    #         for pt in path:
    #             graph.add_node(node_id, coordinates_p=[pt[0], pt[1], pt[2]], radius_p=(major + minor) / 2, active_p=True)
    #             if node_id > 0:
    #                 graph.add_edge(node_id - 1, node_id)
    #             node_id += 1
    #     print("\t-Lava tube graph generated")
    #     processed_graph = self.post_processing_graph(graph_p=graph)
    #     print("\t-Post processing algorithm applied to the graph")
    #     processed_graph.create_adjency_matrix(graph.nb_nodes)
    #     print("\t-Adjency matrix created")
    #     processed_graph.save_graph()
    #     print("\t-Graph saved")
    #     print(f"{Color.BOLD.value}Adjency matrix:{Color.ENDC.value}")
    #     print(graph.adj_matrix)
    #     print(f"{Color.OKBLUE.value} == End of graph {index} generation == {Color.ENDC.value}")
    #     return processed_graph

    def generate_graph(self, index_p):
        """
        Main logic of the graph generation with step-by-step 3D visualization
        """
        index = index_p
        print(f"{Color.OKBLUE.value} == Graph {index} generation begins == {Color.ENDC.value}")
        
        import pyvista as pv
        pv.global_theme.background = 'black'
        pv.global_theme.show_scalar_bar = False

        def show_step(title, volume, opacity=0.7):
            grid = pv.ImageData()
            grid.dimensions = np.array(volume.shape) + 1
            grid.spacing = (1, 1, 1)
            grid.cell_data["values"] = volume.flatten(order="F")
            grid = grid.cell_data_to_point_data()
            mesh = grid.contour(isosurfaces=[0.5])
            p = pv.Plotter(window_size=[800, 600], off_screen=False)
            p.add_mesh(mesh, color='orange', opacity=opacity)
            p.add_title(title, font_size=14)
            p.show(cpos='xy')

        tube = ProceduralLavaTube(
            shape=Config.GENERATION_SIZE.value,
            seed=Config.LT_SEED.value,
            tube_major_radius=Config.LT_TUBE_MAJOR_RADIUS.value,
            tube_minor_radius=Config.LT_TUBE_MINOR_RADIUS.value,
            min_neck_minor=Config.LT_NECK_MINOR_MIN.value,
            chamber_major=Config.LT_CHAMBER_MAJOR_SCALE.value,
            chamber_minor=Config.LT_CHAMBER_MINOR_SCALE.value,
            floor_level_ratio=Config.LT_FLOOR_LEVEL_RATIO.value
        )

        # Step 1: Main path
        tube.generate_main_path(length=Config.LT_MAIN_PATH_LENGTH.value)
        tube.carve_tubes(n_chambers=0, n_necks=0, shape_profiles=["ellipse"])
        show_step("Step 1: Main Tube", tube.volume)
        print("\t-Main path carved and visualized")

        # Step 2: Add side branches
        tube.reset()
        tube.generate_main_path(length=Config.LT_MAIN_PATH_LENGTH.value)
        tube.add_side_branches(
            n_branches=Config.LT_BRANCHES_N.value,
            min_length=Config.LT_BRANCHES_MIN_LENGTH.value,
            max_length=Config.LT_BRANCHES_MAX_LENGTH.value,
            min_offset=Config.LT_BRANCHES_MIN_OFFSET.value,
            max_offset=Config.LT_BRANCHES_MAX_OFFSET.value,
            p_rejoin=Config.LT_BRANCHES_P_REJOIN.value
        )
        tube.carve_tubes(n_chambers=0, n_necks=0, shape_profiles=["ellipse"])
        show_step("Step 2: + Side Branches", tube.volume)
        print("\t-Side branches added and visualized")

        # Step 3: Carve chambers & necks
        tube.reset()
        tube.generate_main_path(length=Config.LT_MAIN_PATH_LENGTH.value)
        tube.add_side_branches(
            n_branches=Config.LT_BRANCHES_N.value,
            min_length=Config.LT_BRANCHES_MIN_LENGTH.value,
            max_length=Config.LT_BRANCHES_MAX_LENGTH.value,
            min_offset=Config.LT_BRANCHES_MIN_OFFSET.value,
            max_offset=Config.LT_BRANCHES_MAX_OFFSET.value,
            p_rejoin=Config.LT_BRANCHES_P_REJOIN.value
        )
        tube.carve_tubes(
            n_chambers=Config.LT_CHAMBERS_N.value,
            n_necks=Config.LT_NECKS_N.value,
            shape_profiles=Config.LT_SHAPE_PROFILES.value,
            rect_min_w=Config.LT_RECT_MIN_WIDTH.value,
            rect_max_w=Config.LT_RECT_MAX_WIDTH.value,
            rect_min_h=Config.LT_RECT_MIN_HEIGHT.value,
            rect_max_h=Config.LT_RECT_MAX_HEIGHT.value,
            crawlspace_h=Config.LT_CRAWLSPACE_HEIGHT.value,
            crevasse_w=Config.LT_CREVASSE_WIDTH.value
        )
        show_step("Step 3: + Chambers & Necks", tube.volume)
        print("\t-Chambers and necks carved and visualized")

        # Step 4: Add roughness
        tube.add_surface_roughness(
            amount=Config.LT_ROUGHNESS_AMOUNT.value,
            repeats=Config.LT_ROUGHNESS_REPEATS.value
        )
        show_step("Step 4: + Surface Roughness", tube.volume, opacity=0.9)
        print("\t-Roughness applied and visualized")

        # Step 5: Final smooth
        tube.smooth(sigma=Config.LT_SMOOTH_SIGMA.value)
        show_step("Step 5: Final (Smoothed)", tube.mask)
        print("\t-Smoothed and visualized")

        # Continue with graph creation
        graph = Graph(self.name, index, Config.MAX_CREATED_NODE_ON_CIRCLE.value)
        node_id = 0
        for name, path, major, minor in tube.paths:
            for pt in path:
                graph.add_node(node_id, coordinates_p=[pt[0], pt[1], pt[2]], radius_p=(major + minor) / 2, active_p=True)
                if node_id > 0:
                    graph.add_edge(node_id - 1, node_id)
                node_id += 1
        print("\t-Lava tube graph generated")
        processed_graph = self.post_processing_graph(graph_p=graph)
        print("\t-Post processing algorithm applied to the graph")
        processed_graph.create_adjency_matrix(graph.nb_nodes)
        print("\t-Adjency matrix created")
        processed_graph.save_graph()
        print("\t-Graph saved")
        print(f"{Color.BOLD.value}Adjency matrix:{Color.ENDC.value}")
        print(graph.adj_matrix)
        print(f"{Color.OKBLUE.value} == End of graph {index} generation == {Color.ENDC.value}")
        return processed_graph

    def post_processing_graph(self, graph_p):
        """
        Various post process solition applied to the graph. Currently:
        - Remove edge duplicates
        """
        # Remove duplicates
        graph = graph_p
        for node in graph.nodes:
            graph.nodes[node].set_edges(Tools.remove_duplicate_none_list(graph.nodes[node].get_edges()))
        
        return graph
    

    def create_graph_picture(self, path_p, saving_path_p):
        """
        Display the created graph
        """
        print(f"\n{Color.OKBLUE.value} == Graph picture generation == {Color.ENDC.value}")
        display = Display(data_path=path_p, voxel_size=0.6,
            node_radius=1.0,      # You can set per-node radii if desired
            edge_radius=1.0,      # You can set per-edge radii if desired
            smoothing=True,
            n_iter=50,
            relaxation_factor=0.1)
        print("\t-Display object created")
        display.load_graph()
        print("\t-Graph important features imported")
        display.voxelize()
        print("\t-Graph voxelized")
        display.extract_surface()
        print("\t-Graph surface extracted")
        
        if Config.ANIMATE.value:
            # Animate the graph
            print("\t-Starting graph animation")
            display.animate_bone_then_mesh_with_orbit(
                path=saving_path_p + "/cave_bone_then_mesh.mp4",
                tube_color="navy",
                mesh_opacity=1,
                n_skip_bone=1,    # Skip edges for faster bone growth animation
                n_skip_mesh=1,    # Skip for faster mesh growth (adjust as you want)
                orbit_frames=36,
                orbit_factor=1.3,)
            print("\t-Graph animated and saved as a video")

            
        if Config.GENERATE_GRAPH_IMAGE.value:
            # Create a static image of the graph
            display.create_static_image(saving_path_p + "/cave_graph.png", tube_color="navy", mesh_opacity=1)
            print("\t-Graph static image created")


        print("\t-Graph animated")
        if self.visualization:
            # Open the graph in a new window
            display.plot()
            print("\t-Graph displayed")
        # display.plot()
        print("\t-Graph plotted")     
        print(f"{Color.OKBLUE.value} == End of graph picture generation == {Color.ENDC.value}")


    def create_mesh(self, index_p, graph_path_p=None):
        """
        Create the mesh using Blender
        """
        # blender_path = Tools.find_file("blender")
        blender_path = "/home/syn/blender-4.4.3-linux-x64/blender"
        index = index_p

        print(f"\n{Color.OKBLUE.value} == Mesh generation start == {Color.ENDC.value}")
        result = None
        try:
            if Config.DEBUG.value:
                result = subprocess.run(f"{blender_path} --python src/blender.py -- -g {graph_path_p} -index {index} -name {self.name}", shell=True, check=True)
            else:
                result = subprocess.run(f"{blender_path} --background --python src/blender.py -- -g {graph_path_p} -index {index} -name {self.name}", shell=True, check=True)
        
        except Exception as e:
            print(f"\n{Color.FAIL.value}An issue occured: ",e)
            print(f"The blender path might be wrong, please check the path.json file")
            print(f"If it is the case, please remove the path.json file{Color.ENDC.value}")
            exit()

        finally:
            if result:
                success = result.stdout
                if success:
                    print(f"{Color.OKGREEN.value}Success: ", success,f"{Color.ENDC.value}")
                
                error = result.stderr
                if error:
                    print(f"{Color.FAIL.value}Error: ", error,f"{Color.ENDC.value}")
            duration = (time.time() - self.starting_time) / 60
            print("Duration of the generation: ", duration," minutes")
            print(f"\n{Color.OKBLUE.value} == Mesh generation finished == {Color.ENDC.value}")
        
        return duration




if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                                description="PLUME project. Procedural Lava-Tube Underground Modeling Engine: A generator that uses procedural generation techniques and graph algorithms to create detailed and visually appealing lava tube structures. ",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    
    parser.add_argument("-n", help="Name of the current graph generation", type=str)
    parser.add_argument("-g", help="Take an already generated graph as input", type=str)

    args = parser.parse_args()
    arguments = vars(args)
    generator = Generator(name_p=arguments['n'], graph_path_p=arguments['g'])