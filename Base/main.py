#%%
import IntersectionControl
import matplotlib.pyplot as plt



inter_ID = 'N0'
lane_num, loop_num = 16, 12
sig_num = [0, 2, 4, 6]
sig_phase = [48, 22, 47, 21]
max_episode = 200

IC = IntersectionControl.IntersectionControl(inter_ID, lane_num, loop_num, sig_num, sig_phase)
throughput_list = IC.start_ep(max_episode)

plt.plot(throughput_list)

# %%
