import time
import numpy as np
import viser
from yourdfpy import URDF
from viser.extras import ViserUrdf


def create_robot_control_sliders(
    server: viser.ViserServer, viser_urdf: ViserUrdf
) -> tuple[list[viser.GuiInputHandle[float]], list[float]]:
    """为每个关节创建滑块，并且实时更新URDF模型"""
    slider_handles: list[viser.GuiInputHandle[float]] = []
    initial_config: list[float] = []
    for joint_name, (lower, upper) in viser_urdf.get_actuated_joint_limits().items():
        # 设置关节的运动范围
        lower = lower if lower is not None else -np.pi
        upper = upper if upper is not None else np.pi
        initial_pos = 0.0 if lower < 0 and upper > 0 else (lower + upper) / 2.0

        # 创建滑块控件
        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=1e-3,
            initial_value=initial_pos,
        )

        # 当滑块值更新时，更新URDF配置
        slider.on_update(
            lambda _: viser_urdf.update_cfg(
                np.array([slider.value for slider in slider_handles])
            )
        )
        
        # 添加滑块和初始位置
        slider_handles.append(slider)
        initial_config.append(initial_pos)

    return slider_handles, initial_config


def main(robot_urdf_path: str = "assets/robots/xarm6/xarm6_wo_ee.urdf") -> None:
    # 启动 Viser 服务器
    server = viser.ViserServer()

    # 加载 URDF 文件，使用yourdfpy库
    urdf = URDF.load(robot_urdf_path)

    # 使用 VisorUrdf 加载解析后的URDF
    viser_urdf = ViserUrdf(server, urdf_or_path=urdf)

    # 在 GUI 中创建控制关节位置的滑块
    with server.gui.add_folder("Joint position control"):
        (slider_handles, initial_config) = create_robot_control_sliders(
            server, viser_urdf
        )

    # 设置初始机器人配置
    viser_urdf.update_cfg(np.array(initial_config))

    # 创建复位按钮
    reset_button = server.gui.add_button("Reset")

    @reset_button.on_click
    def _(_):
        for s, init_q in zip(slider_handles, initial_config):
            s.value = init_q

    # 保持程序运行
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    main("assets/robots/xarm6/xarm6_wo_ee.urdf")  # 指定URDF文件路径