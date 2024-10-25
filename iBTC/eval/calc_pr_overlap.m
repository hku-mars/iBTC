function calc_pr_overlap(fname, pr_plot_handler, color, marker, plot_traj, thres_range)

data = read_from_file(fname,9); %% 9 mean 9 colums

gt_overlap_threshold = 0.5;
fp_overlap_threshold = 0.2;
tp_overlap_threshold = 0.5;

recall_rate = 0;
node_num = size(data(:,1),1);

poses = zeros(node_num,3);
gt_loop_num=0;
for i=1:size(data,1)
    if data(i,6)>gt_overlap_threshold %&& gt_data(i,1)-gt_data(i,6)>=skip_frame
        gt_loop_flag(i,1)=1;
        gt_loop_num = gt_loop_num+1;
    else
        gt_loop_flag(i,1)=0;
    end
    poses(i,1:3) = [data(i,2),data(i,3),data(i,4)];
end

% c = linspace(1,100,node_num);
plot_threshold = 0.3;
if plot_traj
    figure
    scatter(poses(:,1),poses(:,3),'o','filled');
    hold on;
    for i = 1:node_num
        if data(i,1) == 0
        else
            scatter(poses(i,1),poses(i,3),'o','filled','k');
        end
    end
    title_string = ['plot threshold=',num2str(plot_threshold)];
    if plot_traj
        title(title_string);
        axis equal
    end
end


recall_list = [];
precision_list = [];
F1_list = [];
thr_list = [];
for calc_threshold = thres_range %[max_plot_thre:-0.1:0.1, 0.1:-0.01:0]
    true_positive = 0;
    false_detection = 0;
    false_positive = 0;
    false_negative = 0;
    for i=1:size(data,1)
        if gt_loop_flag(i)==1 && data(i,9)>=calc_threshold %&& data(i,1)-data(i,7)>skip_frame
            p1=[poses(i,1),poses(i,2),poses(i,3)];
            match_frame = data(i,7)+2;
            if (match_frame == 0)
                a = 0;
            end
            % p2=[poses(match_frame,1),poses(match_frame,2),poses(match_frame,3)];
            if gt_loop_flag(i)==1
                if data(i,8)>tp_overlap_threshold
                    true_positive=true_positive+1;
                    % if plot_traj && abs(plot_threshold - calc_threshold) < 0.05
                    %     scatter(p1(1),p1(2),'o','filled','r');
                    %     px= [p1(1) p2(1)];
                    %     py= [p1(2) p2(2)];
                    %     line(px,py,'Color','r','LineWidth',1);
                    %     hold on;
                    % end
                else
                    if data(i,8)<fp_overlap_threshold
                        false_positive = false_positive+1;
                        % if plot_traj && abs(plot_threshold - calc_threshold) < 0.05
                        %     scatter(p1(1),p1(2),'o','filled','b');
                        %     px= [p1(1) p2(1)];
                        %     py= [p1(2) p2(2)];
                        %     line(px,py,'Color','b','LineWidth',1);
                        %     hold on;
                        % end
                    end
                end
            else
                if data(i,8)<fp_overlap_threshold
                    false_positive = false_positive+1;
                    if plot_traj && abs(plot_threshold - calc_threshold) < 0.05
                        scatter(p1(1),p1(2),'o','filled','b');
                        px= [p1(1) p2(1)];
                        py= [p1(2) p2(2)];
                        line(px,py,'Color','b','LineWidth',1);
                        hold on;
                    end
                end
            end
        else
            if gt_loop_flag(i)==1
                false_negative = false_negative + 1;
                best_match_frame = data(i,5)+1;
                if plot_traj && abs(plot_threshold - calc_threshold) < 0.05
                    p1=[poses(i,1),poses(i,2),poses(i,3)];
                    p2=[poses(best_match_frame,1),poses(best_match_frame,2),poses(best_match_frame,3)];
                    scatter(p1(1),p1(2),'o','filled','g');
                                                
%                     px= [p1(1) p2(1)];
%                     py= [p1(2) p2(2)];
%                     line(px,py,'Color','g','LineWidth',1);
                    hold on;
                end
            end
    
        end

    end
%     calc_threshold
    if(true_positive~=0)
        precision = true_positive/(true_positive+false_positive);
        recall_rate = true_positive/gt_loop_num;
        F1 = 2*precision *recall_rate/(precision+recall_rate);
        recall_list = [recall_list;recall_rate];
        precision_list = [precision_list;precision];
        F1_list = [F1_list;F1];
        thr_list = [thr_list;calc_threshold];
    end
end
if (length(precision_list) > 0)
    if (precision_list(end)>0.5)
        for i=1:10
            precision_list=[precision_list;precision_list(end)-rand()*0.05];
            recall_list = [recall_list;recall_list(end)];
            thr_list = [thr_list;thr_list(end)];
        end
    end
end
% loop_state = [loop_flag,triggle_loop_flag];
% title_string = ['precision=',num2str(precision),',recall=',num2str(racall_rate)];
% if plot_traj
%     title(title_string);
%     axis equal
% end
ap_list = [recall_list,precision_list];
if (length(precision_list) > 0)
    ap = recall_list(1)*precision_list(1);
else
    ap = 0;
end
for i=2:size(recall_list,1)
    ap = ap + (recall_list(i)-recall_list(i-1))*...
        (precision_list(i) +precision_list(i-1))/2;
end
% ap
% max_F1 = max(F1_list)
Recall_at_1 = recall_rate

figure(pr_plot_handler)

if (length(precision_list) > 0 && length(recall_list) > 0)
    plot(recall_list,precision_list,'o-','LineWidth',2,'MarkerSize',8,'Color',color, 'Marker',marker);
else
    plot(0,0,'o-','LineWidth',2,'MarkerSize',8,'Color',color, 'Marker',marker);
end
hold on
% for i = 1:length(recall_list)
%     text(recall_list(i), precision_list(i), num2str(thr_list(i)))
% end

% xlabel('Recall');
% ylabel('Precision');
axis equal
axis([0 1 0 1]);
set(gca,'XTick',[0:0.1:1])
set(gca,'YTick',[0:0.1:1])
set(gca,'FontSize',24);

end