function val = is_red(rgb)

    if (rgb(1) == 255) && (rgb(2) == 0) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end