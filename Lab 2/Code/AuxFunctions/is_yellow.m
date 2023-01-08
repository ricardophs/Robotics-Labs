function val = is_yellow(rgb)

    if (rgb(1) == 255) && (rgb(2) == 255) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end