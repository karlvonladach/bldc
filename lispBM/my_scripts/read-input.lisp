(plot-init "time" "input")
(plot-add-graph "HALL1")
(plot-add-graph "HALL2")

(gpio-configure 'pin-rx 'pin-mode-in-pu)
(gpio-configure 'pin-tx 'pin-mode-in-pu)

(looprange i 0 5000
    (progn
        (plot-set-graph 0)
        (plot-send-points (/ i 100.0) (gpio-read 'pin-rx))
        (plot-set-graph 1)
        (plot-send-points (/ i 100.0) (gpio-read 'pin-tx))
        (sleep 0.001)
    )
)
