function out = tov(tot_v_state, name)
    out = zeros(length(tot_v_state),size(tot_v_state{1}.(name),2));
    for ind =1 : length(tot_v_state)
        out(ind,:) = tot_v_state{ind}.(name);
    end
end