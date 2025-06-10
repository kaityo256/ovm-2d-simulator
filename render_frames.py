import glob
import yaml
import numpy as np
import matplotlib.pyplot as plt


def load_box_size(config_file="conf.yaml"):
    with open(config_file, "r") as f:
        conf = yaml.safe_load(f)
    return conf["LX"], conf["LY"]


def load_frame_data(filename):
    return np.loadtxt(filename)


def render_frame(data, LX, LY, output_file, dot_size=10, vector_scale=0.5):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_facecolor("white")
    ax.set_xlim(0, LX)
    ax.set_ylim(0, LY)
    ax.set_aspect("equal")
    ax.axis("off")

    x = data[:, 0]
    y = data[:, 1]
    vx = data[:, 2]
    vy = data[:, 3]

    # 黒丸
    ax.scatter(x, y, color="black", s=dot_size)

    # 速度ベクトル
    ax.quiver(
        x,
        y,
        vx,
        vy,
        angles="xy",
        scale_units="xy",
        scale=1 / vector_scale,
        color="black",
        width=0.002,
    )

    plt.savefig(output_file, dpi=150, bbox_inches="tight", pad_inches=0)
    plt.close()


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--dot-size", type=float, default=10, help="agent dot size")
    parser.add_argument(
        "--vector-scale", type=float, default=0.5, help="velocity vector scale"
    )
    args = parser.parse_args()

    LX, LY = load_box_size()
    frame_files = sorted(glob.glob("frame*.dat"))

    for f in frame_files:
        data = load_frame_data(f)
        png_file = f.replace(".dat", ".png")
        render_frame(
            data,
            LX,
            LY,
            png_file,
            dot_size=args.dot_size,
            vector_scale=args.vector_scale,
        )
        print(f"Rendered {png_file}")


if __name__ == "__main__":
    main()
