function val = is_white(rgb)

    if (rgb(1) == 255) && (rgb(2) == 255) && (rgb(3) == 255)
        val = true;
    else
        val = false;
    end

end