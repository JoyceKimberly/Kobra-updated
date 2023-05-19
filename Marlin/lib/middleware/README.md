middleware placed here is automatically made available to the platformio environment if enabled.
enable using a entry in the board manifest corrosponding to the middleware (directory) name, for example `build.mw.sd_card`.

## directory structure

middleware directories **must** be structured in the following way:

```
<middleware name>
 |-- src/
 |-- inc/
```
