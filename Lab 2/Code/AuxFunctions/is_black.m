function val = is_black(rgb)

    if (rgb(1) == 0) && (rgb(2) == 0) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end