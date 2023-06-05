function str = string_from_user(prompt,default)

str = input(prompt,'s');
if isempty(str)
    str = default;
end

end