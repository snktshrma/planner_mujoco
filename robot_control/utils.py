import os

def get_model_path():
    return os.getenv('ROBOT_MODEL_PATH', 
                    os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                'franka_emika_panda', 'scene.xml'))

