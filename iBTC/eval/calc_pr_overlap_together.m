clear
clc
close all

seq_name = "hotel1"
title_name = "iBTC hotel1"
% seq_name = "hotel2"
% title_name = "iBTC hotel2"
% seq_name = "office"
% title_name = "iBTC office"
% seq_name = "mall"
% title_name = "iBTC mall"

% seq_name = "r3live_hkumb"
% title_name = "R3LIVE HKU-MB"
% seq_name = "MARS_AMvalley03"
% title_name = "MARS-LVIG AMvalley03"

% seq_name = "newercollege_park"
% title_name = "NewerCollege park"
% seq_name = "newercollege_cloister"
% title_name = "NewerCollege cloister"

% seq_name = "nclt120115"
% title_name = "NCLT120115"
% seq_name = "nclt120526"
% title_name = "NCLT120526"
% seq_name = "nclt120820"
% title_name = "NCLT120820"
% seq_name = "nclt121201"
% title_name = "NCLT121201"

% seq_name = "kitti00"
% title_name = "KITTI00"

pr_plot_handler = figure(2);
hold on

fname = seq_name + "/lc_gt_doc_" + seq_name + ".txt"
calc_pr_overlap(fname, pr_plot_handler, "#7E2F8E", 'o', false, [0.9:-0.1:0.1, 0.1:-0.01:0])
% 
% fname = seq_name + "/lc_gt_doc_" + seq_name + "_norgb.txt"
% calc_pr_overlap(fname, pr_plot_handler, "#0072BD", "x", false, [0.9:-0.1:0.1, 0.1:-0.01:0])
% 
% fname = seq_name + "/lc_gt_doc_ring_" + seq_name + ".txt"
% calc_pr_overlap(fname, pr_plot_handler, "#77AC30", "^", false, [0.5:-0.003:0])
% 
% fname = seq_name + "/lc_gt_doc_orbdbow2_" + seq_name + ".txt"
% calc_pr_overlap(fname, pr_plot_handler, "#D95319", "square", false, [0.9:-0.1:0.1, 0.1:-0.01:0])
% 
% fname = seq_name + "/lc_gt_doc_netvlad_" + seq_name + ".txt"
% calc_pr_overlap(fname, pr_plot_handler, "#EDB120", '*', false, [2.2:-0.1:0.1, 0.1:-0.01:0])

legend( "iBTC", "BTC", "RING++", "ORB-DBoW2", "NetVALD", 'Location', 'southwest', 'FontSize', 15)
title(title_name, 'FontSize', 28)
xlabel("Recall")
ylabel("Precision")

% saveas(pr_plot_handler, seq_name + "/" + seq_name+ ".png")
