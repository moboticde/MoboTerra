"""Execute real launch factories with recording entities; never start ROS nodes."""
import ast
import os
from pathlib import Path
import warnings
import yaml

from mobotic_config.configuration import Configuration, ConfigurationError, default_platform_path, validate_platform
from mobotic_config.description import render_description

PACKAGES = Path(__file__).resolve().parents[2]


class Entity:
    def __init__(self, kind, *args, **kwargs):
        self.kind, self.args, self.kwargs = kind, args, kwargs

    def perform(self, context):
        if self.kind == 'LaunchConfiguration':
            return context[self.args[0]]
        if self.kind == 'PythonExpression':
            return eval(''.join(p.perform(context) if isinstance(p, Entity) else p
                                for p in self.args[0]), {'__builtins__': {}}, {})
        if self.kind == 'IfCondition':
            result = self.args[0].perform(context)
            return result if isinstance(result, bool) else result.lower() in ('true', '1')
        raise AssertionError(self.kind)


def constructor(kind):
    return lambda *args, **kwargs: Entity(kind, *args, **kwargs)


def environment():
    names = ('LaunchDescription', 'DeclareLaunchArgument', 'GroupAction', 'OpaqueFunction',
             'SetLaunchConfiguration', 'IncludeLaunchDescription', 'IfCondition',
             'AnyLaunchDescriptionSource', 'LaunchConfiguration', 'PathJoinSubstitution',
             'PythonExpression', 'Node', 'PushRosNamespace', 'FindPackageShare', 'ParameterValue')
    env = {name: constructor(name) for name in names}
    env.update(os=os, warnings=warnings, yaml=yaml, Configuration=Configuration,
               ConfigurationError=ConfigurationError, default_platform_path=default_platform_path,
               validate_platform=validate_platform, render_description=render_description)
    path = PACKAGES / 'mobotic_config/mobotic_config/launching.py'
    # Keep function bodies and the real actuator->executable mapping.
    nodes = [n for n in ast.parse(path.read_text()).body
             if isinstance(n, (ast.FunctionDef, ast.Assign))]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), env)
    return env


def factory(path):
    env = environment()
    nodes = [n for n in ast.parse(Path(path).read_text()).body if isinstance(n, ast.FunctionDef)]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), env)
    return env['generate_launch_description']


def resolve(description, overrides=None):
    context = {e.args[0]: str(e.kwargs['default_value']) for e in description.args[0]
               if e.kind == 'DeclareLaunchArgument'}
    context.update(overrides or {})
    actions = []
    pending = list(description.args[0])
    while pending:
        entity = pending.pop(0)
        if entity.kind == 'OpaqueFunction':
            pending[0:0] = entity.kwargs['function'](context, *entity.kwargs.get('args', []))
        elif entity.kind == 'SetLaunchConfiguration':
            context[entity.args[0]] = entity.args[1]
        actions.append(entity)
    return actions, context
