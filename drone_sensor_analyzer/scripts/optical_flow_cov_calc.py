import math
import pandas as pd
import numpy as np

import argparse

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
from torch import norm


def main(args):
    df = pd.read_csv(args.filename)
    df["No"] = range(0, len(df.index))

    # calc K constant
    df["K"] = (
        np.sqrt(df["velocity_x"] ** 2 + df["velocity_y"] ** 2)
        * df["integration_time"]
        / (df["range"] * np.sqrt(df["delta_px"] ** 2 + df["delta_py"] ** 2))
    )

    # calc velocity magnitude
    df["velocity"] = np.sqrt(df["velocity_x"] ** 2 + df["velocity_y"] ** 2)

    # calc delta magnitude
    df["delta"] = np.sqrt(df["delta_px"] ** 2 + df["delta_py"] ** 2)

    # remove NaN and inf
    df = df.replace([np.inf, -np.inf], np.nan).dropna()

    # remove zero velocity
    df = df[df["velocity"] != 0]

    # remove zero delta
    df = df[df["delta"] != 0]

    # add histogram of K constant
    plt.hist(
        df[df["surface_quality"] > 230]["K"], bins=100, alpha=0.5, color="red", log=True
    )

    k_hist, k_bin_edges = np.histogram(df[df["surface_quality"] > 230]["K"], bins=100)
    k_classvalues = (k_bin_edges[:-1] + k_bin_edges[1:]) / 2

    k_list = np.array(list(zip(k_classvalues, k_hist)))
    # sort list by frequency
    k_list = k_list[k_list[:, 1].argsort()[::-1]]

    # print top 5 K constant
    print(f"Top 5 K constant: {k_list[:5]}")

    # determine K constant, use top 5 K constant, weighted by frequency
    k_constant = np.average(k_list[:5, 0], weights=k_list[:5, 1])
    print(f"K constant: {k_constant}")

    """
    # print mode
    k_modes = df[df["surface_quality"] > 230]["K"].mode()
    print(f"K modes: {k_modes}")

    # remove 0 from mode
    k_modes = k_modes[k_modes != 0]

    # determine K constant, use top 3 modes
    k_constant = k_modes.head(1).mean()
    print(f"K constant: {k_constant}")
    """

    plt.show()

    # show K constant
    plt.scatter(
        df[df["surface_quality"] > 230]["No"], df[df["surface_quality"] > 230]["K"]
    )
    plt.axhline(y=k_constant, color="r", linestyle="-")
    plt.show()

    # separate data to 16 bins based on surface quality
    # surface quality is an integer from 0 to 255
    # 255 is the best quality, 0 is the worst
    # 16 bins are created
    df_bins = []
    for i in range(16):
        df_bins.append(
            df[
                (df["surface_quality"] >= (i * 16))
                & (df["surface_quality"] < ((i + 1) * 16))
            ]
        )

    centerized_df_bins = []

    # centerize delta_px and delta_py
    for df_bin in df_bins:
        centerized_df_bins.append(
            df_bin.assign(
                delta_px=lambda x: x["delta_px"]
                - x["velocity_x"] * x["integration_time"] / x["range"] / k_constant,
                delta_py=lambda x: x["delta_py"]
                - x["velocity_y"] * x["integration_time"] / x["range"] / k_constant,
                delta=lambda x: x["delta"]
                - x["velocity"] * x["integration_time"] / x["range"] / k_constant,
                actual_delta_px=lambda x: x["velocity_x"]
                * x["integration_time"]
                / x["range"]
                / k_constant,
                actual_delta_py=lambda x: x["velocity_y"]
                * x["integration_time"]
                / x["range"]
                / k_constant,
            )
        )

        # drop NaN and inf
        centerized_df_bins[-1] = (
            centerized_df_bins[-1].replace([np.inf, -np.inf], np.nan).dropna()
        )

    # calculate covariance for each bin
    covs = []
    for df_bin in centerized_df_bins:
        covs.append(np.cov(df_bin["delta_px"], df_bin["delta_py"]))

    # plot 16 bins
    # add scatter plot of delta_px and delta_py, and histogram of delta_px and delta_py
    hist_axs = []
    fig, axs = plt.subplots(4, 4, figsize=(20, 20))
    for i in range(16):
        axs[i // 4, i % 4].scatter(
            centerized_df_bins[i]["delta_px"], centerized_df_bins[i]["delta_py"]
        )
        axs[i // 4, i % 4].set_title(
            f"surface_quality: {i*16} - {(i+1)*16}, \n cov: {covs[i]}, \n actual dpx: {centerized_df_bins[i]['actual_delta_px'].mean()}"
        )
        axs[i // 4, i % 4].set_xlabel("delta_px")
        axs[i // 4, i % 4].set_ylabel("delta_py")
        axs[i // 4, i % 4].set_xlim([-1, 1])
        axs[i // 4, i % 4].set_ylim([-1, 1])
        axs[i // 4, i % 4].grid()

        hist_axs.append(axs[i // 4, i % 4].twinx())

        hist_axs[i].hist(
            centerized_df_bins[i]["delta_px"], bins=20, alpha=0.5, color="red"
        )
        hist_axs[i].hist(
            centerized_df_bins[i]["delta_py"], bins=20, alpha=0.5, color="blue"
        )

    # add space between plots
    fig.tight_layout(pad=5.0)

    # show plot
    plt.show()

    fig, axs = plt.subplots(4, 4, figsize=(20, 20), subplot_kw={"projection": "3d"})

    # plot 3d histogram of delta, velocity, separated by surface quality
    for i in range(4):
        for j in range(4):
            ax = axs[i, j]

            # Create histogram bins
            delta_bins = np.linspace(-1.0, 1.0, 20)
            velocity_bins = np.linspace(0, 0.2, 20)

            df_bin = centerized_df_bins[i * 4 + j]

            # Create histogram
            hist, xedges, yedges = np.histogram2d(
                df_bin["delta"],
                df_bin["velocity"],
                bins=(delta_bins, velocity_bins),
            )
            # Normalize each velocity bin
            for k in range(hist.shape[1]):
                total = np.sum(hist[:, k])
                if total > 0:
                    hist[:, k] /= total

            # Construct arrays for the anchor positions of the bars.
            xpos, ypos = np.meshgrid(xedges[:-1], yedges[:-1], indexing="ij")
            xpos = xpos.flatten()
            ypos = ypos.flatten()
            zpos = 0

            # Construct arrays with the dimensions for the bars.
            dx = dy = 0.1
            dz = hist.flatten()

            ax.bar3d(xpos, ypos, zpos, dx, dy, dz, shade=True)
            ax.set_xlabel("delta")
            ax.set_ylabel("velocity")
            ax.set_zlabel("Frequency")

            # set z range
            ax.set_zlim([0, 1.0])

    plt.show()

    # plot 2d histogram of delta, velocity, separated by surface quality
    fig, axs = plt.subplots(4, 4, figsize=(20, 20))
    for i in range(4):
        for j in range(4):
            ax = axs[i, j]
            ax.set_title(
                f"surface_quality: {(i*4+j)*16} - {(i*4+j+1)*16}, \n cov: {covs[i*4+j]}, \n actual dpx: {centerized_df_bins[i*4+j]['actual_delta_px'].mean()}"
            )

            df_bin = centerized_df_bins[i * 4 + j]

            ax.hist2d(
                df_bin["delta"],
                df_bin["velocity"],
                bins=[25, 10],
                cmap="Blues",
                range=[[-1, 1], [0, 0.2]],
            )
            ax.set_xlabel("delta")
            ax.set_ylabel("velocity")
    fig.tight_layout(pad=5.0)
    plt.show()


if __name__ == "__main__":
    # get filename from command line
    parser = argparse.ArgumentParser(
        description="Calculate covariance of optical flow measurements."
    )
    parser.add_argument(
        "filename", type=str, help="CSV file containing optical flow measurements."
    )
    args = parser.parse_args()

    main(args)
