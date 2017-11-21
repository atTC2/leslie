# Sample Starting States

The following can be copied and pasted into the `"starting_state"` field in `leslie.json` to begin the system at various initial states.

## AT_HOME (default)
```
{
    "id": "AT_HOME",
    "data": {}
}
```

## LISTENING\_FOR_TABLE
```
{
    "id": "LISTENING_FOR_TABLE",
    "data": {
        "current_owner": "Tom"
    }
}
```

## MOVE\_TO_TABLE
```
{
    "id": "MOVE_TO_TABLE",
    "data": {
        "tableID": 0,
        "current_owner": "Tom",
        "friend": null
    }
}
```

## AT_TABLE
```
{
    "id": "AT_TABLE",
    "data": {
        "current_owner": "Tom",
        "friend": null
    }
}
```

## LOCKED\_AND_WAITING
*Should not be used as an initial state! Use `AT_TABLE`*

## ALARM
*Should not be used as an initial state! Use `AT_TABLE`*

## ALARM_REPORT
*Should not be used as an initial state! Use `AT_TABLE`*

## MOVE\_TO_HOME
```
{
    "id": "MOVE_TO_HOME",
    "data": {}
}
```
