function [state,out] = simplesync(bag,topics,state)

if isempty(state)
    state = [];
    state.data = {}
    state.present = zeros(length(topics),1);
end

while bag.hasNext()
    [msg, meta] = bag.read();

    ind=find(ismember(topics,meta.topic));
    state.present(ind) = 1;
    state.data{ind} = msg;
    if sum(state.present) == length(topics)
        % return
        out = state.data;
        state.data = {};
        state.present(:) = 0;
        return
    end
end
% done
state = {};
out = {};