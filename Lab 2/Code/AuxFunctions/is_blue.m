function val = is_blue(rgb)

    if (rgb(1) == 0) && (rgb(2) == 0) && (rgb(3) == 255)
        val = true;
    else
        val = false;
    end

end