function val = is_green(rgb)

    if (rgb(1) == 0) && (rgb(2) == 255) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end