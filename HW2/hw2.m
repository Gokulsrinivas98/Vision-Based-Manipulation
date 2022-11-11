%Gokul Srinivasan
clc
close all
force_vector_rotations = zeros(3,3,4);
[force_vector_rotations(:,:,1),force_vector_rotations(:,:,2),force_vector_rotations(:,:,3),force_vector_rotations(:,:,4)] = deal([1,0,0; 0,1,0; 0,0,1], ...
    [-1,0,0; 0,-1,0; 0,0,1],[-1,0,0; 0,-1,0; 0,0,1],[0,-1,0; 1,0,0; 0,0,1]);
[object_center, contact_locations] = deal( [3, 1.5],[3,0; 4,3; 2,3; 0,2]);
% Defining rotations for ease of describing vectors
[R1,R2,R3,R4] = deal(force_vector_rotations(:,:,1),[0,1,0; -1,0,0; 0,0,1],force_vector_rotations(:,:,2),force_vector_rotations(:,:,4));
%====================================== bottom horizontal edge ======================================
[bh_msv_values, edge_1_x] = deal([],[0:0.1:6]);
for i = edge_1_x
    [current_contacts,current_force_rotations] = deal(cat(1, contact_locations, [i,0]),cat(3, force_vector_rotations, R1));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    bh_msv_values(end+1) = min(sqrt(eig(G'*G)));
end
%====================================== right vertical edge ======================================
[rv_msv_values,edge_2_x] = deal([],[0:0.1:3]);
for i = edge_2_x
    [current_contacts,current_force_rotations]= deal(cat(1, contact_locations, [6,i]),cat(3, force_vector_rotations, R2));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    rv_msv_values(end+1) = min(sqrt(eig(G'*G)));
end
%====================================== top horizontal edge ======================================
[th_msv_values,edge_3_x] = deal([],[6:-0.1:0]);
for i = edge_3_x
    [current_contacts,current_force_rotations] =deal( cat(1, contact_locations, [i,3]),cat(3, force_vector_rotations, R3));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    th_msv_values(end+1) = min(sqrt(eig(G'*G)));
end
%====================================== left vertical edge ======================================
[lv_msv_values,edge_4_x] = deal([],[3:-0.1:0]);
for i = edge_4_x
    [current_contacts,current_force_rotations] = deal(cat(1, contact_locations, [0,i]),cat(3, force_vector_rotations, R4));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    lv_msv_values(end+1) = min(sqrt(eig(G'*G)));
end
%plot(edge_1_x,btm_horz_msv_values, edge_2_x,right_vert_msv_values, ...
%    edge_3_x,top_horz_msv_values, edge_4_x,left_vert_msv_values)
x1 = [1:size(bh_msv_values,2)];
x2 = [size(bh_msv_values,2) : size(bh_msv_values,2)+size(rv_msv_values,2)-1];
x3 = [size(bh_msv_values,2)+size(rv_msv_values,2)-1 : size(bh_msv_values,2)+size(rv_msv_values,2)+size(th_msv_values,2)-2];
x4 = [size(bh_msv_values,2)+size(rv_msv_values,2)+size(th_msv_values,2)-2 : size(bh_msv_values,2)+size(rv_msv_values,2)+size(th_msv_values,2)+size(lv_msv_values,2)-3];
plot(x1,bh_msv_values,'b', x2,rv_msv_values,'--', x3,th_msv_values,'-.m', x4,lv_msv_values,':k','LineWidth',2.0)
legend('Edge 1', 'Edge 2','Edge 3', 'Edge 4')
title('MSV values for contact points on each edge')
xlabel('Iteration')
ylabel('Minimum Singular Value')
[Max ,I] =deal(zeros(1,4),zeros(1,4));
[Max(1), I(1)] = max(bh_msv_values);
[Max(2), I(2)] = max(rv_msv_values);
[Max(3), I(3)] = max(th_msv_values);
[Max(4), I(4)] = max(lv_msv_values);
[max_msv, ind] = max(Max);
disp(['Minimum Singular Value = ', num2str(max_msv)])
disp(['Corresponding index = ', num2str(I(ind)), ' in edge ', num2str(ind)])
disp(['Hence corresponding point is : (6,0)'])
%====================================== Volume of the ellipsoid in the wrench space ======================================
%====================================== bottom horizontal edge ======================================
[bh_vew_values, edge_1_x] = deal([],[0:0.1:6]);
for i = edge_1_x
    [current_contacts,current_force_rotations] = deal(cat(1, contact_locations, [i,0]),cat(3, force_vector_rotations, R1));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    bh_vew_values(end+1) = sqrt(det(G'*G));
end
%====================================== right vertical edge ======================================
[rv_vew_values,edge_2_x] = deal([],[0:0.1:3]);
for i = edge_2_x
    [current_contacts,current_force_rotations]= deal(cat(1, contact_locations, [6,i]),cat(3, force_vector_rotations, R2));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    rv_vew_values(end+1) = sqrt(det(G'*G));
end
%====================================== top horizontal edge ======================================
[th_vew_values,edge_3_x] = deal([],[6:-0.1:0]);
for i = edge_3_x
    [current_contacts,current_force_rotations] =deal( cat(1, contact_locations, [i,3]),cat(3, force_vector_rotations, R3));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    th_vew_values(end+1) = sqrt(det(G'*G));
end
%====================================== left vertical edge ======================================
[lv_vew_values,edge_4_x] = deal([],[3:-0.1:0]);
for i = edge_4_x
    [current_contacts,current_force_rotations] = deal(cat(1, contact_locations, [0,i]),cat(3, force_vector_rotations, R4));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    lv_vew_values(end+1) = sqrt(det(G'*G));
end
figure
x1 = [1:size(bh_vew_values,2)];
x2 = [size(bh_vew_values,2) : size(bh_vew_values,2)+size(rv_vew_values,2)-1];
x3 = [size(bh_vew_values,2)+size(rv_vew_values,2)-1 : size(bh_vew_values,2)+size(rv_vew_values,2)+size(th_vew_values,2)-2];
x4 = [size(bh_vew_values,2)+size(rv_vew_values,2)+size(th_vew_values,2)-2 : size(bh_vew_values,2)+size(rv_vew_values,2)+size(th_vew_values,2)+size(lv_vew_values,2)-3];
plot(x1,bh_vew_values,'b', x2,rv_vew_values, '--',x3,th_vew_values,'-.m', x4,lv_vew_values,':k','LineWidth',2.0)
legend('Edge 1', 'Edge 2','Edge 3', 'Edge 4')
title('VEW values for contact points on each edge')
xlabel('Iteration')
ylabel('Volume of the ellipsoid in the wrench space')
[Max ,I] =deal(zeros(1,4),zeros(1,4));
[Max(1), I(1)] = max(bh_vew_values);
[Max(2), I(2)] = max(rv_vew_values);
[Max(3), I(3)] = max(th_vew_values);
[Max(4), I(4)] = max(lv_vew_values);
[max_vew, ind] = max(Max);
disp('======================================')
disp(['Maximum Volume of Ellipsoid in Wrench space = ', num2str(max_vew)])
disp(['Corresponding index = ', num2str(I(ind)), ' in edge ', num2str(ind)])
disp(['Hence corresponding point is : (6,0)'])
%====================================== Grasp isotropy index ======================================
%====================================== bottom horizontal edge ======================================
[bh_gii_values,edge_1_x] = deal([],[0:0.1:6]);
for i = edge_1_x
    [current_contacts,current_force_rotations] = deal(cat(1, contact_locations, [i,0]),cat(3, force_vector_rotations, R1));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    bh_gii_values(end+1) = min(sqrt(eig(G'*G)))/max(sqrt(eig(G'*G)));
end
%====================================== right vertical edge ======================================
[rv_gii_values,edge_2_x] = deal([],[0:0.1:3]);
for i = edge_2_x
    [current_contacts,current_force_rotations]= deal(cat(1, contact_locations, [6,i]),cat(3, force_vector_rotations, R2));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    rv_gii_values(end+1) = min(sqrt(eig(G'*G)))/max(sqrt(eig(G'*G)));
end
%====================================== top horizontal edge ======================================
[th_gii_values,edge_3_x] = deal([],[6:-0.1:0]);
for i = edge_3_x
    [current_contacts,current_force_rotations] =deal( cat(1, contact_locations, [i,3]),cat(3, force_vector_rotations, R3));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    th_gii_values(end+1) = min(sqrt(eig(G'*G)))/max(sqrt(eig(G'*G)));
end
% ====================================== left vertical edge ======================================
[lv_gii_values,edge_4_x] = deal([],[3:-0.1:0]);
for i = edge_4_x
    [current_contacts,current_force_rotations] = deal(cat(1, contact_locations, [0,i]),cat(3, force_vector_rotations, R4));
    G = getGraspMatrix(object_center, current_contacts, current_force_rotations);
    lv_gii_values(end+1) = min(sqrt(eig(G'*G)))/max(sqrt(eig(G'*G)));
end
figure
x1 = [1:size(bh_gii_values,2)];
x2 = [size(bh_gii_values,2) : size(bh_gii_values,2)+size(rv_gii_values,2)-1];
x3 = [size(bh_gii_values,2)+size(rv_gii_values,2)-1 : size(bh_gii_values,2)+size(rv_gii_values,2)+size(th_gii_values,2)-2];
x4 = [size(bh_gii_values,2)+size(rv_gii_values,2)+size(th_gii_values,2)-2 : size(bh_gii_values,2)+size(rv_gii_values,2)+size(th_gii_values,2)+size(lv_gii_values,2)-3];
plot(x1,bh_gii_values,'b', x2,rv_gii_values,'--', x3,th_gii_values,'-.m ',x4,lv_gii_values,':k','LineWidth',2.0)

legend('Edge 1', 'Edge 2','Edge 3', 'Edge 4')
title('GII values for contact points on each edge')
xlabel('Iteration')
ylabel('Grasp Isotrophy Index')
[Max ,I] =deal(zeros(1,4),zeros(1,4));
[Max(1), I(1)] = max(bh_gii_values);
[Max(2), I(2)] = max(rv_gii_values);
[Max(3), I(3)] = max(th_gii_values);
[Max(4), I(4)] = max(lv_gii_values);
[max_vew, ind] = max(Max);
disp('======================================')
disp(['Maximum Grasp Isotrophy Index = ', num2str(max_vew)])
disp(['Corresponding index = ', num2str(I(ind)), ' in edge ', num2str(ind)])
disp(['Hence corresponding point is : (3.8,0)'])

% ====================================== FUNCTION DEFINITION ======================================
function graspMatrix = getGraspMatrix(object_center, current_contacts, current_force_rotations)   
    no_of_contacts_rows = size(current_contacts, 1);
    % no_of_contacts_cols = size(contact_locations, 2);
    G = [];
    H = [1, 0, 0; 0, 1, 0; 0, 0, 0];
    for i = 1:no_of_contacts_rows
       [a1,a2] = deal(current_contacts(i,1) - object_center(1),current_contacts(i,2) - object_center(2));
       [Pi,R] = deal([1, 0, -a2; 0, 1, a1; 0, 0, 1],current_force_rotations(:,:,i));
       Gi = H*R*Pi;
       G = vertcat(G, Gi);
    end
    graspMatrix = G;
end


